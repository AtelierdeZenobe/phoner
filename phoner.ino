// Tests done
// - check deepleep -> update some code, lib
// - check SIM in sleep -> if do not define CALL_FRED, that's ok !!
//                         if        define CALL_FRED, that's not ok !! I may get the call but SIM800L does not sleep after
// It's so weird, not stable from test to test
// Definetely need to check answer from SIM, retry if not the expected answer
// Ok it works !

// RST pin is HIGH -> SIM on
// DTR pin is LOW  -> com' UART active (used for sleep mode 1)
// use Sleep mode 2
// put some delay when sending AT CFUN and CSCLK to make sure I got an OK from sim

/// Deepsleep
//#include "esp_sleep.h"
#include "driver/rtc_io.h" // see https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
/// Sim800l comm
#include <HardwareSerial.h>

#define CALL_FRED // comment or not to make a fake call or call fred

#define USE_SIMPLE_DELAY true // uncomment for the new code for ATCommand, otherwise, use OLD CODE with SIMPLE DELAY
#define DELAY_WAIT_SIM 5000       
#define SHORT_DELAY_WAIT_SIM 200  
#define TIMEOUT_SIM 3000          // used if USE_SIMPLE_DELAY = false

#define SLEEP_MODE 2 // 1 (using DTR pin) or 2

#define PHONER_BOARD_V1
//#define PHONER_BOARD_V2 // must be tested !!

#ifdef PHONER_BOARD_V1
const int RX_PIN = 4;
const int TX_PIN = 5;
/// Button
#define BTN_GPIO GPIO_NUM_6 // PIN 6/D12 = LP GPIO for wake up ESP32 /!\ used for RED LED on PCB
// These pins are connected on the PCB, but not used for the moment => define as inputs
const int RX_PIN_UNUSED_ON_BOARD = 16;
const int TX_PIN_UNUSED_ON_BOARD = 17;
/// RST pin - routed on PCB
#define RST_GPIO GPIO_NUM_7
/// DTR pin - external wire
#define DTR_GPIO GPIO_NUM_19
// LED
#define LED GPIO_NUM_23
#endif

#ifdef PHONER_BOARD_V2 // everything routed on PCB
const int RX_PIN = 4;
const int TX_PIN = 5;
/// Button
#define BTN_GPIO GPIO_NUM_6 // PIN 6/D12 = LP GPIO for wake up ESP32
/// RST pin
#define RST_GPIO GPIO_NUM_21
/// DTR pin
#define DTR_GPIO GPIO_NUM_7
// RGB LED
#define LED_R GPIO_NUM_20
#define LED_G GPIO_NUM_19
#define LED_B GPIO_NUM_16
// LED
#define LED GPIO_NUM_23

#endif

// Serial HW com' with SIM800L
HardwareSerial sim800l(1); // RX, TX pins defined above

/// Battery
const int VBAT_PIN = 0;
const float VREF = 3.3;

/// On-board led
const int led = 15;

/// Setup logic
RTC_DATA_ATTR int bootCount = 0;

// need function prototype
bool sendATCommand(HardwareSerial &serial, const char *message,const char *command, const char *expectedResponse, unsigned long timeout, bool use_delay, unsigned long delay_value);

void setup()
{ 
  /*
   * INITIAL SETUP
  */

  blink(500,1);
  /// On-board led
  pinMode(led, OUTPUT);
  pinMode(RST_GPIO,OUTPUT);
  pinMode(DTR_GPIO,OUTPUT);
  
  //// Define RX-TX pins on PCB as input to not interfere with pin 4 and 5 finally used with wires for the moment
  pinMode(RX_PIN_UNUSED_ON_BOARD,INPUT);
  pinMode(TX_PIN_UNUSED_ON_BOARD,INPUT);
  
  /// Btn
  pinMode(BTN_GPIO, INPUT);          // I made it as a pullup button, so use PULLDOWN internal resistor with rtc functions (see after call deep_sleep)
                                     // change input arg of deep sleep function esp_sleep_enable_ext1_wakeup accordingly
                                     // it seems that a pullup button is better, the wake up condition on HIGH input is better
  
  /// Battery check
  analogReadResolution(12);

  // setup SIM800L pin digital level
  digitalWrite(DTR_GPIO,LOW);
  digitalWrite(RST_GPIO,HIGH);
  delay(100);

  // Setup Serial communications
  Serial.begin(9600); // Serial monitor comm
  Serial.write("Serial initialized\n");

  // Setup sim800l module
  Serial.write("Initializing SIM800L ...");
  sim800l.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Works with HardwareSerial(1)
  Serial.write("done\n");
  delay(1000); // need a delay here
}

void loop()
{
  // Bootcount counts the number of boot :D
  Serial.print("Bootcount = ");
  Serial.println(bootCount);
  blink(1000,bootCount);
  
  if (bootCount == 0)
  {
    start_sim800L();
    // Blink LED at the end of boot sequence
    blink(200,3);
  }

  bootCount++;
  if(bootCount > 0 )
  {
    wake_up_sim800L();
    call();
  }

  sleep_sim800L();
  sleep_esp32();
  Serial.println("This will never be printed");
}

// ------------------------------------------------------------- //
// ESP32 functions
// ------------------------------------------------------------- //
// sleep_esp32()

void sleep_esp32()
{
  // TODO: use ANY_LOW if pull-up
  // https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
  esp_sleep_enable_ext1_wakeup((1ULL << BTN_GPIO), ESP_EXT1_WAKEUP_ANY_HIGH);
  // esp_sleep_enable_ext0_wakeup(BTN_GPIO, 1); // wake up if BTN pressed 
                                                // idt does not work ?? https://github.com/espressif/esp-idf/issues/11932
  rtc_gpio_pulldown_en(BTN_GPIO);  // GPIO6 is tie to GND in order to wake up in HIGH
  rtc_gpio_pullup_dis(BTN_GPIO);   // Disable PULL_UP in order to allow it to wakeup on HIGH
  // reminder : COM port is disabled, so board does show disconnected in IDE
  esp_deep_sleep_start();
}

// ------------------------------------------------------------- //
// SIM800L functions
// ------------------------------------------------------------- //
// sendATCommand()
// start_sim800L()
// reset_sim800L()
// sleep_sim800L()
// wake_up_sim800L()
// call()

// Send and check AT Commeand
// use : sendATCommand(sim800l,"Message for serial Monitor","AT","OK",DELAY_SLEEP_SIM)
bool sendATCommand(HardwareSerial &serial, const char *message,const char *command, const char *expectedResponse, unsigned long timeout = TIMEOUT_SIM, bool use_delay = USE_SIMPLE_DELAY, unsigned long delay_value = DELAY_WAIT_SIM) {
    serial.println(command);
    if (use_delay)
    {
      updateSerial();
      delay(delay_value);
      return true;
    }
    else
    {
      unsigned long startTime = millis();
      String response = "";

      while (millis() - startTime < timeout) {
          while (serial.available()) {
              char c = serial.read();
              response += c;
              // Vérifiez si la réponse attendue est contenue
              if (response.indexOf(expectedResponse) != -1) {
                  Serial.println(String(message) + " Command " + String(command) + "->\n" + response);
                  return true;
              }
          }
          delay(1);
      }
      Serial.println(String(message) + " Command " + String(command) + " failed:\n" + response);
      return false;
      // What to do now ?

    }
}

void start_sim800L()
{
  sendATCommand(sim800l,"Starting handshake...","AT","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,DELAY_WAIT_SIM); // Starting handshake
  sendATCommand(sim800l,"Signal quality test...","AT+CSQ","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM); //Signal quality test, value range is 0-31 , 31 is the best
  sendATCommand(sim800l,"Reading SIM information...","AT+CCID","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM); //Read SIM information to confirm whether the SIM is plugged
  sendATCommand(sim800l,"Selecting mobile operator...","AT+COPS=0,2","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM); // select the mobile network operator| AT+COPS=<mode>(0 for auto),<format>(2 for numeric),<oper>,<AcT>
  // there is an error here
  sendATCommand(sim800l,"Checking network status...","AT+CREG?","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM); //check the network registration status. Will answer +CREG: <n>,<stat>
  sendATCommand(sim800l,"Querying battery status...","AT+CBC","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM); // Querry battery status
    
}
void reset_sim800L()
{
  digitalWrite(RST_GPIO,HIGH);
  delay(200);
  digitalWrite(RST_GPIO,LOW); // put this pin LOW for (at least) 100ms for a hard reset
  delay(200);
  digitalWrite(RST_GPIO,HIGH);
  delay(200);
}
// Sleep mode 2 : https://www.raviyp.com/223-sim900-sim800-sleep-mode-at-commands/
// RF OFF : datasheet page 26, section 4.3
void sleep_sim800L()
{
  Serial.println("");
  Serial.println("Going to sleep now");
  sendATCommand(sim800l,"Set RF off","AT+CFUN=4","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM);
  delay(DELAY_WAIT_SIM); // should reply within 10 sec max according to AT CFUN page 96
  if (SLEEP_MODE == 1)
  {
    sendATCommand(sim800l,"\nSet sleep mode 1","AT+CSCLK=1","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM);
    delay(100);
    digitalWrite(DTR_GPIO,HIGH); // put high for sleep mode
    delay(100);
  }
  else if (SLEEP_MODE == 2)
  {
    sendATCommand(sim800l,"\nSet sleep mode 2","AT+CSCLK=2","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM);
    delay(DELAY_WAIT_SIM); // wait for at least 5s without UART, on air or IO INTR
  }
  else 
  {
    // no SLEEP mode
  }
  
}

void wake_up_sim800L()
{
  Serial.println("Wake up SIM800L");
  sim800l.println("AT"); // dummy data, wait no response, so do not use sendATCommand
  delay(200); // datasheet AT command page 153, note 2 : apply a delay of (at least) 100ms between dummy/waking data and AT command
  Serial.println("\nDisable sleep mode");
  if (SLEEP_MODE == 1)
  {
    digitalWrite(DTR_GPIO,LOW);
    delay(100); // UART should be ready after 50ms
  }
  else if (SLEEP_MODE == 2)
  {
    sendATCommand(sim800l,"\n","AT+CSCLK=0","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM);
    delay(DELAY_WAIT_SIM); // wait for at least 5s without UART, on air or IO INTR
  }
  else 
  {
    // no SLEEP mode
  }
  sendATCommand(sim800l,"\n","AT+CFUN=1","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM);
  delay(DELAY_WAIT_SIM); // wait for a few seconds to let RF be ready
}

void call()
{
  Serial.println("Who do you call ?");
  // CALL
  
#ifdef CALL_FRED
  sendATCommand(sim800l,"\nCalling...","ATD+ +32475896931;","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM);
  // sim800l.println("ATD+ +32475896931;");
  // updateSerial();
#else
  Serial.write("Calling...\n");
  Serial.write("Fake call :) \n");  
#endif
  Serial.print("waiting 20 sec ");
  for (int n_loop=0;n_loop<20;n_loop++)
  {
    delay(1000); // wait
    Serial.print(".");
  }
  Serial.println("");
  // better to wait for a reply or a connection beforing hanging up ??

  
  sendATCommand(sim800l,"\nHanging up...","ATH","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM);
  // Serial.write("Hanging up...\n");
  // sim800l.println("ATH"); //hang up
  // updateSerial();
}

// ------------------------------------------------------------- //
// Misc functions
// ------------------------------------------------------------- //

void updateSerial()
{
  delay(500); // wait for any incoming data
  while (Serial.available()) 
  {
    sim800l.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(sim800l.available()) 
  {
    Serial.write(sim800l.read());//Forward what Software Serial received to Serial Port
  }
}

// blink N times the LED with a period of 2*millis
void blink(const int millis,int N)
{
  for (int i=0;i<N;i++)
  {
    digitalWrite(led, LOW);
    delay(millis);
    digitalWrite(led, HIGH);
    delay(millis);
    digitalWrite(led, LOW);
  }
  
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
    blink(500,1);
  }
  // blink(500,voltage_deca_precise);

  Serial.print("Battery Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");
}

