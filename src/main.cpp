//#define BLYNK_TEMPLATE_ID "TMPLyzvlR6NP"
//#define BLYNK_DEVICE_NAME "Room Monitor"

//#define BLYNK_FIRMWARE_VERSION        "0.1.0"
//#define BLYNK_PRINT Serial
const int led_gpio = 32; // LED light for esp32

// #define USE_NODE_MCU_BOARD

//#include "BlynkEdgent.h"

#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
//#include <BME280I2C.h>
#include <MHZ19.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// ---> if ESP8266 (Software serial needed)
// #include <SoftwareSerial.h> ESP32 has built in UARTs that can be used for hardware serial
// #define RX_PIN 16                                          // Rx pin which the MHZ19 Tx pin is attached to
// #define TX_PIN 17                                          // Tx pin which the MHZ19 Rx pin is attached to
// #define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)
// #define OLED_RESET LED_BUILTIN // Reset pin # ESP-8266
// SoftwareSerial co2Serial(RX_PIN, TX_PIN); // Serial connection with MHZ219

// ---> if ESP32 (Hardware serial builtin with UARTs)
#define OLED_RESET led_gpio // Resest pin for # ESP32
HardwareSerial co2Serial(2);

MHZ19 co2sensor; // Constructor for MHZ219

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Constructor for SSD1306
Adafruit_BME280 bme;
// BME280I2C bme; // Constructor for BME280 alternate library that can read 

unsigned long getDataTimer = 0;
const int ledGreenPin = 19;
const int ledYellowPin = 18;
const int ledRedPin = 5;

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");

  pinMode (ledGreenPin, OUTPUT);
  pinMode (ledYellowPin, OUTPUT);
  pinMode (ledRedPin, OUTPUT);

  // ---> ESP8266
  // co2Serial.begin(BAUDRATE);
  // ---

  // ---> ESP32
  co2sensor.begin(co2Serial);
  // ---

  co2sensor.autoCalibration(); // Turn auto calibration ON (OFF autoCalibration(false))
  Serial.println("co2 sensor started");

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Initialize with the I2C addr 0x3C
  Serial.println("SSD1306 started");
  display.clearDisplay(); // Clear display buffer.
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.println("SSD1306 started");
  display.display();
  sleep(1);

  if (!bme.begin(0x76, &Wire)) { // Initialize BME280 Sensor
    Serial.println("BME280 failed");
    display.println("BME280 failed");
    display.display();
    sleep(1);
    display.clearDisplay(); // Clear display buffer
    while(1); // Will trip WDT and reset
  }
  else {
    Serial.println("BME280 started");
    display.println("BME280 started");
    display.display();
    sleep(1);
    display.clearDisplay(); // Clear display buffer
  }

  Serial.println("Ready");
  display.println("Ready");
  display.display();
  sleep(1);

  //BlynkEdgent.begin();
  display.clearDisplay();
}

void show_display(float temp, float pres, float hum, int co2) {
  display.clearDisplay(); // Clear display buffer
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Temp: " + String(temp) + " C");
  display.drawRect(67, 0, 3, 3, WHITE);
  display.println("Pressure: " + String(pres) + " Pa");
  display.println("Humidity: " + String(hum) + "%\n");
  display.println("CO2: " + String(co2) + " ppm");
  //display.println("eTemp: " + String(etemp) + " C");
  display.display();
}

//void send_to_blynk(float pres, float temp, float hum, float co2, float etemp){
//  Blynk.virtualWrite(V1, pres);
//  Blynk.virtualWrite(V2, temp);
//  Blynk.virtualWrite(V3, hum);
//  Blynk.virtualWrite(V4, co2);
//  Blynk.virtualWrite(V5, etemp);
//}

void loop() {
  //BlynkEdgent.run();
  if (millis() - getDataTimer >= 2000){

    float temp(NAN), pres(NAN), hum(NAN), co2(NAN), etemp(NAN);        
    //int8_t etemp;

    /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even 
    if below background CO2 levels or above range (useful to validate sensor). You can use the 
    usual documented command with getCO2(false) */

    temp = bme.readTemperature(); // Read temperature from BME280
    pres = bme.readPressure();
    hum = bme.readHumidity();
    co2 = co2sensor.getCO2(); // Read CO2 (as ppm)
    etemp = co2sensor.getTemperature(); // Read temperature estimate from MHZ219
  
    show_display(temp, pres, hum, co2);
    //send_to_blynk(pres, temp, hum, co2, etemp);
    if (co2 < 600){
      digitalWrite (ledGreenPin, HIGH);
      digitalWrite (ledYellowPin, LOW);
      digitalWrite (ledRedPin, LOW);
    }
    else if (co2 < 800){
      digitalWrite (ledGreenPin, LOW);
      digitalWrite (ledYellowPin, HIGH);
      digitalWrite (ledRedPin, LOW);      
    }
    else{
      digitalWrite (ledGreenPin, LOW);
      digitalWrite (ledYellowPin, LOW);     
      digitalWrite (ledRedPin, HIGH);    
    }

    getDataTimer = millis();  // Soft sleep
  }
}