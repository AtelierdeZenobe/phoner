/// Deepsleep
#include "esp_sleep.h"

/// Sim800l comm
#include <HardwareSerial.h>
HardwareSerial sim800l(1); // RX, TX
const int RX_PIN = 4;
const int TX_PIN = 5;

// These pins are connected on the PCB, but not used for the moment => define as inputs
const int RX_PIN_UNUSED_ON_BOARD = 16;
const int TX_PIN_UNUSED_ON_BOARD = 17;

/// Button
#define BTN_GPIO GPIO_NUM_6

/// Battery
const int VBAT_PIN = 0;
const float VREF = 3.3;

/// On-board led
const int led = 15;

/// Setup logic
RTC_DATA_ATTR int bootCount = 0;

void setup()
{ 
  /*
   * INITIAL SETUP
   *
  */

  blink(500);
  /// On-board led
  pinMode(led, OUTPUT);

  //// Define RX-TX pins on PCB as input to not interfere with pin 4 and 5 finally used with wires for the moment
  pinMode(RX_PIN_UNUSED_ON_BOARD,INPUT);
  pinMode(TX_PIN_UNUSED_ON_BOARD,INPUT);
  
  /// Btn
  pinMode(BTN_GPIO, INPUT_PULLDOWN);
  
  for(int i=0; i<bootCount; i++)
  {
    blink(1000);
  }
  //bootCount++; // do it later

  /// Battery check
  analogReadResolution(12);

  // Setup Serial communications
  Serial.begin(115200); // Serial monitor comm
  Serial.write("Serial initialized\n");
  Serial.write("Initializing SIM800L ...");
  sim800l.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Works with HardwareSerial(1)
  Serial.write("done\n");

  if (bootCount == 0)
  {
    // Setup sim800l module
    Serial.write("Starting handshake...\n");
    sim800l.println("AT"); //Once the handshake test is successful, it will back to OK
    updateSerial();
    delay(10000);

    Serial.write("Signal quality test...\n");
    sim800l.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
    updateSerial();

    Serial.write("Reading SIM information...\n");
    sim800l.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
    updateSerial();

    Serial.write("Selecting mobile operator...\n");
    sim800l.println("AT+COPS=0,2"); // select the mobile network operator| AT+COPS=<mode>(0 for auto),<format>(2 for numeric),<oper>,<AcT>
    updateSerial();

    Serial.write("Checking network status...\n");
    sim800l.println("AT+CREG?"); //check the network registration status. Will answer +CREG: <n>,<stat>
    updateSerial();

    Serial.write("Querying battery status...\n");
    sim800l.println("AT+CBC"); // Querry battery status
    updateSerial();

    // Blink LED twice at the end of boot sequence
    blink(200);
    blink(200);
    blink(200);
  }

  bootCount++;

  if(bootCount > 0 )
  {
    wake_up_sim800L();
    call();
  }

  Serial.println("Going to sleep now");
  sleep_sim800L();
  // // TODO: use ANY_LOW if pull-up
  // esp_sleep_enable_ext1_wakeup((1ULL << BTN_GPIO), ESP_EXT1_WAKEUP_ANY_HIGH);
  // // reminder : COM port is disabled, so board does show disconnected in IDE
  // esp_deep_sleep_start();
  // Serial.println("This will never be printed");
}

void loop()
{
  // Never called
}

// Sleep mode 2 : https://www.raviyp.com/223-sim900-sim800-sleep-mode-at-commands/
// RF OFF : datasheet page 26, section 4.3
void sleep_sim800L()
{
  Serial.println("Set RF off");
  sim800l.println("AT+CFUN=4"); // RF off
  updateSerial(); // OK is expected
  Serial.println("Set sleep mode 2");
  sim800l.println("AT+CSCLK=2"); // sleep mode 2
  updateSerial(); // OK is expected
  delay(5000); // wait for at least 5s without UART, on air or IO INTR
  // May check using AT+CFUN? and AT+CSCLK?
}

void wake_up_sim800L()
{
  Serial.println("Wake up SIM800L");
  sim800l.println("AT"); // dummy data
  Serial.println("Disable sleep mode 2");
  sim800l.println("AT+CSCLK=0"); // go out from sleep mode 2
  updateSerial(); // OK is expected
  sim800l.println("AT+CFUN=1"); // normal function, wake up RF
  updateSerial(); // OK is expected
  // May check using AT+CFUN? and AT+CSCLK?  
}

void blink(const int millis)
{
  digitalWrite(led, LOW);
  delay(millis);
  digitalWrite(led, HIGH);
  delay(millis);
  digitalWrite(led, LOW);
}

void blinkForBattery()
{
  // ADC is 12 bits precision -> 0 to 4095
  int rawValue = analogRead(VBAT_PIN); // Voltage from ADC [0;4095]
  // Vref is 3.3V so 4095 correspond to 3.3V.
  float voltage = (rawValue * VREF) / 4095.0; // Convert to actual volts (max 3.3V)

  // The actual max battery voltage is around 4.1V
  // The critical battery voltage is below 3.5V
  // The mesurement is actually VBAT/2 => MAX = 2.05 and min = 1.75

  int voltage_deca = voltage * 10/ 3.3; // Scale to [0-10] to blink up to 10 times, representing the current voltage

  // Now blink up to 10 times, showing more precision
  // It blinks 7 times when full. In theory, full = 4.2V -> 2.1V measured -> 0.63/10 -> blink 6 times.
  // It blinks TBC times when on battery for a lil while. In theory 3.7V -> 1.85V measured -> 0.56/10 -> blink 5 times.
  int voltage_deca_precise = (voltage-1) * 10;
  for(int i=0; i < voltage_deca_precise; ++i)
  {
    blink(500);
  }

  Serial.print("Battery Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");
}

void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    sim800l.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(sim800l.available()) 
  {
    Serial.write(sim800l.read());//Forward what Software Serial received to Serial Port
  }
}

void call()
{
  // CALL
  Serial.write("Calling...\n");
  sim800l.println("ATD+ +32475896931;");
  updateSerial();
  delay(20000); // wait for 20 seconds

  Serial.write("Hanging up...\n");
  sim800l.println("ATH"); //hang up
  updateSerial();
}
