// Tests done
// - check deepleep -> update some code, lib
// - check SIM in sleep -> if do not define CALL_FRED, that's ok !!
//                         if        define CALL_FRED, that's not ok !! I may get the call but SIM800L does not sleep after
// It's so weird, not stable from test to test
// Definetely need to check answer from SIM, retry if not the expected answer ?
// Ok it works by adjusting delays !

// RST pin is HIGH -> SIM on
// DTR pin is LOW  -> com' UART active (used for sleep mode 1)
// use Sleep mode 2
// put some delay when sending AT CFUN and CSCLK to make sure I got an OK from sim

/// Deepsleep
//#include "esp_sleep.h"
#include "driver/rtc_io.h" // see https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
/// Sim800l comm
#include <HardwareSerial.h>

const char* PHONE_NUMBER = "+32498341934";

#define USE_SIMPLE_DELAY false // set "false" for the new code for ATCommand, otherwise, set "true" to  use OLD CODE with SIMPLE DELAY
#define DELAY_WAIT_SIM 5000       
#define SHORT_DELAY_WAIT_SIM 200  
#define TIMEOUT_SIM 15000          // used if USE_SIMPLE_DELAY = false

#define SLEEP_MODE 2 // 1 (using DTR pin) or 2

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
// RGB LED
#define LED_R GPIO_NUM_19
#define LED_G GPIO_NUM_16 // actually routed on pin6 but BTN is plugged on that pin finally
#define LED_B GPIO_NUM_20
// LED2
#define LED2 GPIO_NUM_23

// Serial HW com' with SIM800L
HardwareSerial sim800l(1); // RX, TX pins defined above

/// Battery
const int VBAT_PIN = 0; // IO0 through a a res bridge (VBAT--1M--IO0--1M//100nF--GND) as in SCH
const float VREF = 3.3;

/// On-board led
const int led = 15;

/// Setup logic
RTC_DATA_ATTR int bootCount = 0;

enum OPERATION_RESULT
{
  ONGOING,
  OK,
  TIMEOUT,
  WRONG_ANSWER
};

// need function prototype
OPERATION_RESULT sendATCommand(HardwareSerial &serial, const char *message,const char *command, const char *expectedResponse, unsigned long timeout, bool use_delay, unsigned long delay_value);

void setup()
{ 
  /*
   * INITIAL SETUP
  */

  blink(500,1);
  /// On-board led
  pinMode(led, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  
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
  pinMode(VBAT_PIN, INPUT);

  // setup SIM800L pin digital level
  digitalWrite(DTR_GPIO,LOW);
  digitalWrite(RST_GPIO,HIGH);
  delay(100);

  // Setup Serial communications
  Serial.begin(9600); // Serial monitor comm
  Serial.write("Serial initialized\n");
  Serial.write("Use Phoner Board v1.1 \n");
  

  // Setup sim800l module
  Serial.write("Initializing SIM800L ...");
  sim800l.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Works with HardwareSerial(1)
  Serial.write("done\n");
  delay(1000); // need a delay here
}

void loop()
{
  Serial.print("Bootcount = ");
  Serial.println(bootCount);
  blink(500,bootCount); // for debug purpose
  blinkForBattery();
  if (bootCount++ == 0)
  {
    start_sim800L();
    // Blink LED at the end of boot sequence
    blink(100,3);
  }
  else
  {
    wake_up_sim800L();
    if(!call())
    {
      blink_RGB(500, 3, HIGH, LOW, LOW);
    }
  }

  sleep_sim800L();
  blinkForBattery(); // check battery before sleep
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
OPERATION_RESULT sendATCommand(HardwareSerial &serialSIM, const char *message,const char *command, const char *expectedResponse, unsigned long timeout = TIMEOUT_SIM, bool use_delay = USE_SIMPLE_DELAY, unsigned long delay_value = DELAY_WAIT_SIM)
{
  OPERATION_RESULT result = OPERATION_RESULT::ONGOING;
  serialSIM.println(command);
  if (use_delay)
  {
    updateSerial();
    delay(delay_value);
    result = OPERATION_RESULT::OK;
  }
  else
  {
    unsigned long startTime = millis();
    unsigned long elapsedTime = 0;
    String response = "";

    while ( result==OPERATION_RESULT::ONGOING )
    {
      elapsedTime =  millis() - startTime;
      if( elapsedTime > timeout )
      {
        result = OPERATION_RESULT::TIMEOUT;
      }
      while (serialSIM.available() && result==OPERATION_RESULT::ONGOING)
      {
        char c = serialSIM.read();
        response += c;
        // Vérifiez si la réponse attendue est contenue
        if (response.indexOf(expectedResponse) != -1)
        {
          result = OPERATION_RESULT::OK;
        }
      }
      //delay(1); // it is ok because baudrate is set to 9600x
    }

    switch(result)
    {
      case OPERATION_RESULT::OK:
      {
        Serial.println(String(message) + " Command " + String(command) + " success");
        Serial.println(response);
        Serial.println(elapsedTime);
        break;
      }
      case OPERATION_RESULT::ONGOING:
      {
        Serial.println(String(message) + " Command " + String(command) + " ONGOING");
        Serial.println(response);
        break;
      }
      case OPERATION_RESULT::TIMEOUT:
      {
        Serial.println(String(message) + " Command " + String(command) + " TIMEOUT");
        Serial.println(response);
        Serial.println(elapsedTime);
        break;
      }
      case OPERATION_RESULT::WRONG_ANSWER:
      {
        Serial.println(String(message) + " Command " + String(command) + " WRONG ANSWER");
        Serial.println(response);
        break;
      }
      default:
      {
        Serial.println(String(message) + " Command " + String(command) + " DEFAULT");
        Serial.println(response);
        break;
      }
    }     
    return result;
  }
}

void start_sim800L()
{
  sendATCommand(sim800l,"Starting handshake...","AT","OK"); // Starting handshake
  sendATCommand(sim800l,"Signal quality test...","AT+CSQ","OK"); //Signal quality test, value range is 0-31 , 31 is the best
  sendATCommand(sim800l,"Reading SIM information...","AT+CCID","OK"); //Read SIM information to confirm whether the SIM is plugged
  //sendATCommand(sim800l,"Selecting mobile operator...","AT+COPS=0,2","OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM); // select the mobile network operator| AT+COPS=<mode>(0 for auto),<format>(2 for numeric),<oper>,<AcT>
  // there is an error here, is it worth sending this command ? So comment it
  sendATCommand(sim800l,"Checking network status...","AT+CREG?","OK"); //check the network registration status. Will answer +CREG: <n>,<stat>
  sendATCommand(sim800l,"Querying battery status...","AT+CBC","OK"); // Querry battery status
    
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
  sendATCommand(sim800l,"Set RF off","AT+CFUN=4","OK");
  delay(DELAY_WAIT_SIM); // should reply within 10 sec max according to AT CFUN page 96
  if (SLEEP_MODE == 1)
  {
    sendATCommand(sim800l,"Set sleep mode 1","AT+CSCLK=1","OK");
    delay(100);
    digitalWrite(DTR_GPIO,HIGH); // put high for sleep mode
    delay(100);
  }
  else if (SLEEP_MODE == 2)
  {
    sendATCommand(sim800l,"Set sleep mode 2","AT+CSCLK=2","OK");
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
  Serial.println("Disable sleep mode");
  if (SLEEP_MODE == 1)
  {
    digitalWrite(DTR_GPIO,LOW);
    delay(100); // UART should be ready after 50ms
  }
  else if (SLEEP_MODE == 2)
  {
    sendATCommand(sim800l,"Wake up module SIM","AT+CSCLK=0","OK");
    delay(DELAY_WAIT_SIM); // wait for at least 5s without UART, on air or IO INTR
  }
  else 
  {
    // no SLEEP mode
  }
  sendATCommand(sim800l,"Wake up RF","AT+CFUN=1","OK");
  delay(DELAY_WAIT_SIM); // wait for a few seconds to let RF be ready
}

bool call()
{
  Serial.println("Who do you call ?");
  // CALL

  OPERATION_RESULT call_success;
  OPERATION_RESULT hang_success;

  if (strlen(PHONE_NUMBER) >= 9)
  {
    call_success=sendATCommand(sim800l,"Calling...",(String("ATD") + PHONE_NUMBER + ";").c_str(),"OK",TIMEOUT_SIM,USE_SIMPLE_DELAY,SHORT_DELAY_WAIT_SIM);
  }
  else
  {
    Serial.write("Size of phone number < 9 => Fake call...\n");
    call_success=OPERATION_RESULT::OK; 
  }

  Serial.print("waiting 20 sec ");
  for (int n_loop=0;n_loop<20;n_loop++)
  {
    delay(1000); // wait
    Serial.print(".");
  }
  Serial.println("");
  // better to wait for a reply or a connection beforing hanging up ??
  hang_success=sendATCommand(sim800l,"Hanging up...","ATH","OK");
  // here, check if call and/or hanging are true
  if ( (hang_success != OPERATION_RESULT::OK) && (call_success != OPERATION_RESULT::OK)) // a problem occurred -> blink or restart call ?
    return false;
  else
    return true;
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

// blink N times the LED with a period of 2*millis
void blink_RGB(const int millis,int N, unsigned char state_LEDR, unsigned char state_LEDG, unsigned char state_LEDB)
{
  for (int i=0;i<N;i++)
  {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    
    delay(millis);
    digitalWrite(LED_R, state_LEDR);
    digitalWrite(LED_G, state_LEDG);
    digitalWrite(LED_B, state_LEDB);
    
    delay(millis);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
  }
}

void blinkForBattery()
{
  // ADC is 12 bits precision -> 0 to 4095
  // int rawValue = analogRead(VBAT_PIN); // Voltage from ADC [0;4095]
  int NMES = 4; // no more than 16
  unsigned int rawValue=0;
  for(int i=0;i<NMES;i++)
  {
    rawValue+=analogRead(VBAT_PIN); // Voltage from ADC [0;4095]
    delay(1);
  }
  rawValue = rawValue/NMES; // average of a bunch of measures
  
  // Vref is 3.3V so 4095 correspond to 3.3V.
  float voltage = 2 * (rawValue * VREF) / 4095.0; // Convert to actual volts (max 3.3V)
  
  Serial.print("Battery Voltage: ");
  Serial.print(voltage); // VBAT = measured_voltage * 2
  Serial.println(" V");
  
  // use the RGD LED instead
  // Green  : >50%    of [3.5-4.2] : voltage>3.85 V
  // Yellow : 25-50%  of [3.5-4.2] : 3.675 V <= voltage <= 3.85 V 
  // Red    : <25%    of [3.5-4.2] : voltage < 3.675 V

  // TODO: temporary: led RGB is only R due to short
  // Theorical minimal voltage is 3.4V but the batery measurement seems lower than actual battery ? TBC
  if ( (voltage) < 2.9 ) //Yolo
  {
    blink_RGB(300,5,HIGH,LOW,LOW);
  }
}

