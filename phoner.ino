#define SIM800L_RESET_PIN 4
#define SIM800L_TX_PIN 11
#define SIM800L_RX_PIN 10

#include <HardwareSerial.h>

HardwareSerial sim800l(1); // RX, TX

void setup()
{
  //Begin serial communication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(115200);
  
  //Begin serial communication with Arduino and SIM800L
  sim800l.begin(9600, SERIAL_8N1, 4, 5); // Works with HardwareSerial(1)

  Serial.println("\n\nInitializing...");

  sim800l.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  delay(10000);
  sim800l.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  sim800l.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  sim800l.println("AT+COPS=0,2"); //Check whether it has registered in the network
  updateSerial();
  sim800l.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();
  sim800l.println("AT+CBC"); //Check whether it has registered in the network
  updateSerial();
  //sim800l.println("AT+COPS?"); //Check whether it has registered in the network
  //updateSerial();
  sim800l.println("ATD+ +32498341934;"); //  change ZZ with country code and xxxxxxxxxxx with phone number to dial
  updateSerial();
  delay(20000); // wait for 20 seconds...
  sim800l.println("ATH"); //hang up
  updateSerial();
}

void loop()
{

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