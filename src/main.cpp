#include <Adafruit_SSD1306.h>
#include <BME280I2C.h>
#include <BlynkSimpleEsp8266.h>
#include <MHZ19.h>
#include <SoftwareSerial.h>

#define RX_PIN D3                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN D4                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

MHZ19 co2sensor;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial

unsigned long getDataTimer = 0;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET LED_BUILTIN // Reset pin #

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BME280I2C bme;

const char *ssid = "ssid";
const char *pass = "pass";

const char blynk_auth[] = "VI0YXm-pivgiFESgYX4noYhsrbfrFt5Q"; //"Bedroom"
int alarm;

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");

//  pinMode(13, OUTPUT);

  mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
  co2sensor.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 

  co2sensor.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);    // initialize with the I2C addr 0x3C
  display.clearDisplay();  // Clear the buffer.

  // Display Text on OLED
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();

  if (!bme.begin()) {
    Serial.println("BME280 failed");
    display.println("BME280 failed");
    display.display();
    while(1);
  }
  else {
    display.println("BME280 started");
    display.display();
  }

  Blynk.begin(blynk_auth, ssid, pass);
  display.clearDisplay();
}

void loop() {
  if (millis() - getDataTimer >= 2000){
    Blynk.run();
        
    int co2;

    /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even 
    if below background CO2 levels or above range (useful to validate sensor). You can use the 
    usual documented command with getCO2(false) */

    co2 = co2sensor.getCO2(); // Request CO2 (as ppm)
    
    Serial.print("CO2 (ppm): ");
    Serial.println(co2);

    int8_t etemp;
    etemp = co2sensor.getTemperature(); // Request Temperature (as Celsius)

    float temp(NAN), hum(NAN), pres(NAN);

    bme.read(pres, temp, hum);
  
    Blynk.virtualWrite(V1, pres);
    Blynk.virtualWrite(V2, temp);
    Blynk.virtualWrite(V3, hum);

    Blynk.virtualWrite(V4, co2);