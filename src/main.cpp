#include <Adafruit_SSD1306.h>
#include <BME280I2C.h>
#include <BlynkSimpleEsp8266.h>
#include <MHZ19.h>
#include <SoftwareSerial.h>
#include "config.cpp" // Configuration for WiFi and Blynk

#define RX_PIN D5                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN D6                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET LED_BUILTIN // Reset pin #

#define calibrationButton D8
#define calibrationPin D4

MHZ19 co2sensor; // Constructor for MHZ219
SoftwareSerial co2Serial(RX_PIN, TX_PIN); // Serial connection with MHZ219
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Constructor for SSD1306
BME280I2C bme; // Constructor for BME280

unsigned long loopTimer = 0;
unsigned long calibrationTimer = 0;

void show_display(float pres, float temp, float hum, float co2, float etemp) {
  display.clearDisplay(); // Clear display buffer
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Temp: " + String(temp) + " C");
  display.drawRect(67, 0, 3, 3, WHITE);
  display.println("Pressure: " + String(pres) + " Pa");
  display.println("Humidity: " + String(hum) + "%\n");
  display.println("CO2: " + String(co2) + " ppm");
  display.println("eTemp: " + String(etemp) + " C");
  display.display();
}

void send_to_blynk(float pres, float temp, float hum, float co2, float etemp){
  Blynk.virtualWrite(V1, pres);
  Blynk.virtualWrite(V2, temp);
  Blynk.virtualWrite(V3, hum);
  Blynk.virtualWrite(V4, co2);
  Blynk.virtualWrite(V5, etemp);
}

ICACHE_RAM_ATTR void calibrate_mhz19(){
  Serial.println("Button Pressed.");
  co2sensor.autoCalibration(false); // Turn auto calibration ON (OFF autoCalibration(false))
  digitalWrite(calibrationPin, LOW);
  calibrationTimer = millis();
}

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");

  co2Serial.begin(BAUDRATE);
  co2sensor.begin(co2Serial);
  co2sensor.autoCalibration(); // Turn auto calibration ON (OFF autoCalibration(false))

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Initialize with the I2C addr 0x3C
  display.clearDisplay(); // Clear display buffer.
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();

  if (!bme.begin()) { // Initialize BME280 Sensor
    Serial.println("BME280 failed");
    display.println("BME280 failed");
    display.display();
    while(1); // Will trip WDT and reset
  }
  else {
    display.println("BME280 started");
    display.display();
  }

  Blynk.begin(blynk_auth, ssid, pass);
  //display.clearDisplay();

  attachInterrupt(digitalPinToInterrupt(calibrationButton), calibrate_mhz19, RISING);
}

void loop() {
  //if (millis() - loopTimer >= 10000){
    Blynk.run();  // Establish WiFi and link to Blynk

    float temp(NAN), hum(NAN), pres(NAN);        
    int co2;
    int8_t etemp;

    /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even 
    if below background CO2 levels or above range (useful to validate sensor). You can use the 
    usual documented command with getCO2(false) */

    co2 = co2sensor.getCO2(); // Read CO2 (as ppm)
    etemp = co2sensor.getTemperature(); // Read temperature estimate from MHZ219
    bme.read(pres, temp, hum); // Read data from BME280
  
    show_display(pres, temp, hum, co2, etemp);
    send_to_blynk(pres, temp, hum, co2, etemp);

    if ((digitalRead(calibrationPin) == LOW) && (millis() - calibrationTimer >= 8000)) {      
      digitalWrite(calibrationPin, HIGH);
      co2Serial.begin(BAUDRATE);
      co2sensor.begin(co2Serial);
      co2sensor.autoCalibration(false);
    }
    delay(8000);
    //loopTimer = millis();  // Soft sleep
    //ESP.deepSleep(4e6);
  //}
}