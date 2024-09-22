/// Sim800l comm
#include <HardwareSerial.h>
HardwareSerial sim800l(1); // RX, TX
const int RX_PIN = 4;
const int TX_PIN = 5;

/// Button
const int buttonPin = 9;
bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 500;

/// Battery
const int VBAT_PIN = 0;
const float VREF = 3.3;

/// On-board led
const int led = 15;

void setup()
{ 
  /// On-board led
  pinMode(led,OUTPUT);

  /// Battery check
  analogReadResolution(12);

  // Setup Serial communications
  Serial.begin(115200); // Serial monitor comm
  sim800l.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Works with HardwareSerial(1)

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

  // Setup button and interupt
  pinMode(buttonPin, INPUT_PULLUP);  // Set the button pin as input with internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);
}

void loop()
{
  // Check if the button was pressed
  if (buttonPressed) {
    Serial.println("Button Pressed!");
    call();
    blinkForBattery();
    buttonPressed = false;  // Call() is blocking so all button press that happened during call() are ignored
  }
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
  sim800l.println("ATD+ +32498341934;");
  updateSerial();
  delay(20000); // wait for 20 seconds

  Serial.write("Hanging up...\n");
  sim800l.println("ATH"); //hang up
  updateSerial();
}

void buttonISR() 
{
  unsigned long currentTime = millis();
  // Debounce logic
  if((currentTime - lastDebounceTime) > debounceDelay)
  {
    buttonPressed = true;
    lastDebounceTime = currentTime;
  }
}