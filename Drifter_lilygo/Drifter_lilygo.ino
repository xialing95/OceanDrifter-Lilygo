/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  Attention! Please check out TinyGSM guide:
    https://tiny.cc/tinygsm-readme

  Change GPRS apm, user, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!

 *************************************************************/

/* Fill-in your Template ID (only if using Blynk.Cloud) */
#define BLYNK_TEMPLATE_ID "TMPL2vzn6V7jN"
#define BLYNK_TEMPLATE_NAME "OceanDrifterB1"
#define BLYNK_AUTH_TOKEN "iYXD-U_FUdAoTATQjeHTHhS0Q6c7eBzN"

// Select your modem:
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// Default heartbeat interval for GSM is 60
// If you want override this value, uncomment and set this option:
//#define BLYNK_HEARTBEAT 30

#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>

#include <Arduino.h>
#include <Wire.h>

BlynkTimer timer;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = BLYNK_AUTH_TOKEN;

// Your GPRS credentials
// Leave empty, if missing user or pass
char apn[]  = "super";
char user[] = "";
char pass[] = "";

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT  Serial1
#define UART_BAUD   115200
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4
#define LED_PIN     12
#define BAT_ADC     35
#define TEMP_ADC    32
bool reply = false;

TinyGsm modem(SerialAT);

//BLYNK_WRITE(V3)
//{
//    if (param.asInt() == 1) {
//
//        digitalWrite(LED_PIN, LOW);
//        Blynk.logEvent("led_off");//Sending Events
//    } else {
//        digitalWrite(LED_PIN, HIGH);
//        Blynk.logEvent("led_on");//Sending Events
//    }
//}
//
////Syncing the output state with the app at startup
//BLYNK_CONNECTED()
//{
//    Blynk.syncVirtual(V3);  // will cause BLYNK_WRITE(V3) to be executed
//}

float readBattery(uint8_t pin)
{
    int vref = 1100;
    uint16_t volt = analogRead(pin);
    float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
    return battery_voltage;
}

float readTemp(uint8_t pin)
{
  uint16_t volt = analogRead(pin);
  float resistance = 10000/ ((4095.0 / volt)  - 1);     //10K / (1023/ADC - 1) 
  float temp; 
  temp = resistance / 10000;     // (R/Ro)  
  temp = log(temp);                  // ln(R/Ro)  
  temp /= 3950;                   // 1/B * ln(R/Ro)  
  temp += 1.0 / (25 + 273.15); // + (1/To)  
  temp = 1.0 / temp;                 
  temp -= 273.15;                         
  return temp;
}

// This function sends Arduino's up time every second to Virtual Pin (5).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.
void sendGPS()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  float mv = readBattery(BAT_ADC);
  float temp = readTemp(TEMP_ADC);
  Blynk.virtualWrite(V0, ((mv / 4200) * 100));
  SerialMon.println("Battery: " + String(mv));
  Blynk.virtualWrite(V3, temp);
  SerialMon.println("Temperature: " + String(temp));
  
  // Set SIM7000G GPIO4 HIGH ,turn on GPS power
  // CMD:AT+SGPIO=0,4,1,1
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    SerialMon.println(" SGPIO=0,4,1,1 false ");
  }

  modem.enableGPS();
  delay(15000);

  float lat      = 0;
  float lon      = 0;
  float speed    = 0;
  float alt      = 0;
  int   vsat     = 0;
  int   usat     = 0;
  float accuracy = 0;
  int   year     = 0;
  int   month    = 0;
  int   day      = 0;
  int   hour     = 0;
  int   min      = 0;
  int   sec      = 0;

  if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy,
                   &year, &month, &day, &hour, &min, &sec)) {
      Blynk.virtualWrite(V1, double(lon), double(lat));
      SerialMon.println("Latitude: " + String(lat, 8) + "\tLongitude: " + String(lon, 8));
  } 
  else {
      SerialMon.println("No GPS Signal");
  }

  //SerialMon.println("Retrieving GPS/GNSS/GLONASS location again as a string");
  //String gps_raw = modem.getGPSraw();
  //Blynk.virtualWrite(V2, gps_raw);
  //SerialMon.println("GPS/GNSS Based Location String: " + gps_raw);
  
  SerialMon.println("Disabling GPS");
  modem.disableGPS();

  // Set SIM7000G GPIO4 LOW ,turn off GPS power
  // CMD:AT+SGPIO=0,4,1,0
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1) {
    SerialMon.println(" SGPIO=0,4,1,0 false ");
  }

  delay(200);
}



void setup()
{
    Serial.begin(115200); // Set console baud rate
    SerialMon.println("Place your board outside to catch satelite signal");

    delay(100);

    // Set LED OFF
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    
    // Starting the machine requires at least 1 second of low level, and with a level conversion, the levels are opposite
    delay(1000);
    digitalWrite(PWR_PIN, LOW);

//    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
//    if (!SD.begin(SD_CS)) {
//        Serial.println("SDCard MOUNT FAIL");
//    } else {
//        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
//        String str = "SDCard Size: " + String(cardSize) + "MB";
//        Serial.println(str);
//    }

    Serial.println("\nWait...");

    delay(1000);

    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);


    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    Serial.println("Initializing modem...");
    if (!modem.init()) {
        Serial.println("Failed to init modem, attempting to continue without restarting");
    }

    String name = modem.getModemName();
    delay(500);
    Serial.println("Modem Name: " + name);

//    // Launch BMP085
//    if (!bmp.begin()) {
//        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
//        while (1) {}
//    }

    Blynk.begin(auth, modem, apn, user, pass);
    // Setup a function to be called every 25 second
    timer.setInterval(10000L, sendGPS);
}

void loop()
{

    Blynk.run();
    timer.run();

}
