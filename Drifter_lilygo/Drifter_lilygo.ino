/*************************************************************
 MIT-Portugal Marine Robotics Summer Program 2023
 Ocean Drifter Challenge Example Code
 Hardware: LILYGO SIM7000
           BMP390
           10K Thermistor
 Developed by: Charlene Xia
 Based on: https://randomnerdtutorials.com/lilygo-t-sim7000g-esp32-lte-gprs-gps/
           https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G
 ************************************************************* 
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

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
// DEBUG and Sensor Enable Setting 
#define DEBUG 1
#define BMPEnable 0
#define TempEnable 0

/*************************************************************
SIM7000G Modem & Sensor Hardware setup
 *************************************************************/
// Select your modem:
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

/*************************************************************
BLYNK SETUP 
 *************************************************************/
// Double check: Ocean Drifter 1
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
#define BLYNK_TEMPLATE_ID "TMPL2vzn6V7jN"
#define BLYNK_TEMPLATE_NAME "OceanDrifterB1"
#define BLYNK_AUTH_TOKEN "iYXD-U_FUdAoTATQjeHTHhS0Q6c7eBzN"

char auth[] = BLYNK_AUTH_TOKEN;

// Your GPRS credentials
// Leave empty, if missing user or pass
char apn[]  = "super";
char user[] = "";
char pass[] = "";

// Default heartbeat interval for GSM is 60
// If you want override this value, uncomment and set this option:
// #define BLYNK_HEARTBEAT 30

BlynkTimer timer;

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)
float airTemp;
float pressure; 
float altitude;
Adafruit_BMP3XX bmp;

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

/*************************************************************
Sensors Setup Functions
 - BMPsetup()
 *************************************************************/
void BMPsetup(){
    if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
      Serial.println("Could not find a valid BMP390 sensor, check wiring!");
      while (1);
    }
  
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("Adafruit BMP390 setup complete");
}

/*************************************************************
Sensors Functions for reading value
 - readBattery(), read battery voltage
 - readTemp(), read 10K thermistor voltage divider value
 - readBaro(), read temperature, pressure and estimate altitude
 *************************************************************/
float readBattery(uint8_t pin)
{
    int vref = 1100;
    uint16_t volt = analogRead(pin);
    float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
    
    if (DEBUG){
      SerialMon.println("Battery = ");
      SerialMon.println(battery_voltage);
      SerialMon.println(" *V");                   
    }
    
    return battery_voltage;
}

float readTemp(uint8_t pin)
{
  uint16_t volt = analogRead(pin);
  float resistance = 10000/ ((4095.0 / volt)  - 1);     //10K / (1023/ADC - 1) 
  float temp; 
  temp = resistance / 10000;                            // (R/Ro)  
  temp = log(temp);                                     // ln(R/Ro)  
  temp /= 3950;                                         // 1/B * ln(R/Ro)  
  temp += 1.0 / (25 + 273.15);                          // + (1/To)  
  temp = 1.0 / temp;                 
  temp -= 273.15;     

  if (DEBUG){
    SerialMon.println("Water Temperature = ");
    SerialMon.println(temp);
    SerialMon.println(" *C");                   
  }
  
  return temp;
}

void readBaro(){
  if (! bmp.performReading()) {
    SerialMon.println("Failed to perform reading :(");
    return;
  }
  airTemp = bmp.temperature;
  pressure = bmp.pressure / 100.0;
  altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  
  if (DEBUG){
    SerialMon.println("Air Temperature = ");
    SerialMon.println(airTemp);
    SerialMon.println(" *C");

    SerialMon.println("Pressure = ");
    SerialMon.println(pressure);
    SerialMon.println(" hPa");
  
    SerialMon.println("Approx. Altitude = ");
    SerialMon.println(altitude);
    SerialMon.println(" m");
  
    SerialMon.println();
  }
}

/*************************************************************
Sensors Functions for sending value via blynk
 - sentGPS(), read and sentGPS coordinate as double and string 
   for blynk
 - sentData(), call sentGPS() and send read sensor data.
 *************************************************************/
void sentGPS(){
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
      if (DEBUG){
        SerialMon.println("Latitude: " + String(lat, 8) + "\tLongitude: " + String(lon, 8));
      }
  } 
  else {
      SerialMon.println("No GPS Signal");
  }

  SerialMon.println("Retrieving GPS/GNSS/GLONASS location again as a string");
  String gps_raw = modem.getGPSraw();
  Blynk.virtualWrite(V2, gps_raw);
  if (DEBUG){
    SerialMon.println("GPS/GNSS Based Location String: " + gps_raw);
  }
  
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

void sendData(){
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  if (TempEnable){
    float mv = readBattery(BAT_ADC);
    float temp = readTemp(TEMP_ADC);
    Blynk.virtualWrite(V0, ((mv / 4200) * 100));
    Blynk.virtualWrite(V3, int(temp));
  }

  if (BMPEnable){
    delay(1000);
    readBaro();
    Blynk.virtualWrite(V4, double(pressure));
    Blynk.virtualWrite(V5, double(altitude));
  }
  
  delay(1000);
  sentGPS();
  //delay(2000);
}

void setup()
{
    Serial.begin(115200); // Set console baud rate
    SerialMon.println("Place your board outside to catch satelite signal");

    delay(100);

//  Set LED OFF
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);

//  Initialize SIM7000G Modem 
    // Starting the machine requires at least 1 second of low level, and with a level conversion, the levels are opposite
    delay(1500);
    digitalWrite(PWR_PIN, LOW);

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
    Serial.println("Modem Name: " + name + " Setup Completely");

//  Initialize BMP390
    if (BMPEnable){
      Serial.println("Adafruit BMP390 test");
      BMPsetup();
    }
    else{
      Serial.println("BMP not enable.");
    }
    
//  Initialize Blynk
    Serial.println("Trying to connect to Celluar Data & Blynk Platform");
    Blynk.begin(auth, modem, apn, user, pass);
    // Setup a function to be called every 25 second
    timer.setInterval(25000L, sendData);
    SerialMon.println("Blynk Connected & Transmit Timer set");
}

void loop()
{
    Blynk.run();
    timer.run();
}
