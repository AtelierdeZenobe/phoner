#define SIM800L_RESET_PIN 4
#define SIM800L_TX_PIN 11
#define SIM800L_RX_PIN 10

#include <HardwareSerial.h>

HardwareSerial sim800l(1); // RX, TX

const int buttonPin = 9;
bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 500;

void setup()
{
  // Setup Serial communications
  Serial.begin(115200); // Serial monitor comm
  sim800l.begin(9600, SERIAL_8N1, 4, 5); // Works with HardwareSerial(1)

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
    buttonPressed = false;  // Call() is blocking so all button press that happened during call() are ignored
  }
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