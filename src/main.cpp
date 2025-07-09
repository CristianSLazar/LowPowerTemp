// Display Library example for SPI e-paper panels from Dalian Good Display and boards from Waveshare.
// Requires HW SPI and Adafruit_GFX. Caution: the e-paper panels require 3.3V supply AND data lines!
//
// Display Library based on Demo Example from Good Display: https://www.good-display.com/companyfile/32/
//
// Author: Jean-Marc Zingg
//
// Version: see library.properties
//
// Library: https://github.com/ZinggJM/GxEPD2
//
// Purpose: show uses of GxEPD2_GFX base class for references to a display instance

// Supporting Arduino Forum Topics (closed, read only):
// Good Display ePaper for Arduino: https://forum.arduino.cc/t/good-display-epaper-for-arduino/419657
// Waveshare e-paper displays with SPI: https://forum.arduino.cc/t/waveshare-e-paper-displays-with-spi/467865
//
// Add new topics in https://forum.arduino.cc/c/using-arduino/displays/23 for new questions and issues

// see GxEPD2_wiring_examples.h for wiring suggestions and examples

// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 1

// uncomment next line to use class GFX of library GFX_Root instead of Adafruit_GFX
//#include <GFX.h>
// Note: if you use this with ENABLE_GxEPD2_GFX 1:
//       uncomment it in GxEPD2_GFX.h too, or add #include <GFX.h> before any #include <GxEPD2_GFX.h>
// !!!!  ============================================================================================ !!!!

#include <pgmspace.h>

const unsigned char wifiIcon [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x00, 
	0x1f, 0xff, 0xf8, 0x00, 0x00, 0xff, 0xff, 0xfe, 0x00, 0x01, 0xfc, 0x00, 0x7f, 0x80, 0x07, 0xe0, 
	0x00, 0x0f, 0xc0, 0x0f, 0x80, 0x00, 0x03, 0xe0, 0x1f, 0x00, 0x00, 0x00, 0xf8, 0x3c, 0x00, 0xff, 
	0x00, 0x78, 0x38, 0x07, 0xff, 0xe0, 0x38, 0x00, 0x1f, 0xff, 0xf8, 0x00, 0x00, 0x7f, 0x01, 0xfc, 
	0x00, 0x00, 0xfc, 0x00, 0x3e, 0x00, 0x01, 0xf0, 0x00, 0x0f, 0x00, 0x01, 0xe0, 0x00, 0x07, 0x00, 
	0x00, 0x80, 0x7c, 0x02, 0x00, 0x00, 0x01, 0xff, 0x80, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x00, 
	0x0f, 0xc3, 0xe0, 0x00, 0x00, 0x0f, 0x00, 0xe0, 0x00, 0x00, 0x06, 0x00, 0x40, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x7c, 
	0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


#include <GxEPD2_3C.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME280.h"
#include <ESP32Time.h>
#include "esp_timer.h"

#include "BitmapDisplay.h"
#include "TextDisplay.h"
#include "Roboto_Regular16pt7b.h"
#include "roboto_regular48pt.h"

#include <ezTime.h>
#include <WiFi.h>

const char* ssid       = "UPB-Guest";
const char* password   = "";



// select the display constructor line in one of the following files (old style):
#include "GxEPD2_display_selection.h"


// for handling alternative SPI pins (ESP32, RP2040) see example GxEPD2_Example.ino


BitmapDisplay bitmaps(display);




#define SEALEVELPRESSURE_HPA (1013.25)

RTC_DATA_ATTR uint64_t boot_time_unix = 0;  // Unix timestamp
RTC_DATA_ATTR int boot_count = 0;

Adafruit_BME280 bme; // I2C

ESP32Time rtc;

int BatteryLevel;
int prevBatteryLevel = -1;

void showTemp(){
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" ˚C");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();

  display.setRotation(0);

  display.firstPage();

  do {
    display.setFont(&roboto_regular48pt);
    display.setTextColor(GxEPD_BLACK);
    int16_t tbx, tby; uint16_t tbw, tbh;
    // center text
    display.getTextBounds(String(bme.readTemperature() )+ "*C", 0, 0, &tbx, &tby, &tbw, &tbh);
    uint16_t wx = (display.width() - tbw) / 2;
    uint16_t wy = (display.height() / 3) + tbh / 2; // y is base line!
    display.getTextBounds(String(bme.readHumidity() )+ "%", 0, 0, &tbx, &tby, &tbw, &tbh);
    uint16_t fx = (display.width() - tbw) / 2;
    uint16_t fy = (display.height() * 2 / 3) + tbh / 2; // y is base line!

    //display.fillScreen(GxEPD_WHITE);
    display.setCursor(wx, wy);
    display.print(String(bme.readTemperature()));
    display.write(0x7F);
    display.print("C");
    display.setCursor(fx, fy);
    display.print(String(bme.readHumidity() )+ "%");

    display.setFont(&Roboto_Regular16pt7b);
    display.getTextBounds(rtc.getTime("%H:%M %a, %d %b"), 0, 0, &tbx, &tby, &tbw, &tbh);
    uint16_t rx = ( 10);
    uint16_t ry = (tbh + 5) ; // y is base line!

    display.setCursor(rx, ry);
    display.print(rtc.getTime("%H:%M %a, %d %b"));
    display.fillRect( display.width()-80, 10, 61, 35, GxEPD_BLACK );
    display.fillRect( display.width()-78, 12, 56, 30, GxEPD_WHITE);
    display.fillRect( display.width()-19, 18, 7, 20, GxEPD_BLACK );

    if(BatteryLevel >1750 && BatteryLevel <1874 && prevBatteryLevel != 1){
        display.fillRect( display.width()-76, 15, 16, 25, GxEPD_BLACK );
        prevBatteryLevel = 1;
    }else if(BatteryLevel > 1875 && BatteryLevel <1949 && prevBatteryLevel != 2){
        display.fillRect( display.width()-76, 15, 16, 25, GxEPD_BLACK );
        display.fillRect( display.width()-58, 15, 16, 25, GxEPD_BLACK );
        prevBatteryLevel = 2;
    }else if(BatteryLevel >1950 && prevBatteryLevel != 3){
      display.fillRect( display.width()-76, 15, 16, 25, GxEPD_BLACK );
      display.fillRect( display.width()-58, 15, 16, 25, GxEPD_BLACK );
      display.fillRect( display.width()-40, 15, 16, 25, GxEPD_BLACK );
      prevBatteryLevel = 3;
    }
    
    if (boot_count == 1 || ((boot_count - 1) % 1440 == 0) )
      display.drawBitmap(display.width()-130, 12, wifiIcon , 40, 32, GxEPD_BLACK);

    display.setCursor(10, display.height()-10);
    display.print(String(bme.readAltitude(SEALEVELPRESSURE_HPA) )+ "m");

    display.getTextBounds(String(bme.readPressure() / 100.0F )+ " hPa", 0, 0, &tbx, &tby, &tbw, &tbh);
    display.setCursor(display.width() - tbw - 10, display.height()-10);
    display.print(String(bme.readPressure() / 100.0F )+ " hPa");

  } while (display.nextPage());
}

void setup()
{
  Timezone RomaniaTime;
  
  Serial.begin(115200);
  boot_count++;

  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  
  Wire.setPins(15, 23);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // How long we've been asleep
  const uint64_t sleep_duration_us = 60 * 1000000ULL; // 60s
  const uint64_t sleep_duration_s = sleep_duration_us / 1000000ULL;

  // Update time
  if (boot_count == 1 || ((boot_count - 1) % 1440 == 0) ) {
    // First boot: assume an approximate start time (e.g., 0 or set manually)
    WiFi.begin(ssid);
    waitForSync();
    RomaniaTime.setLocation("RO");
    Serial.println("Getting Time fron NTP Server");
    Serial.print(RomaniaTime.dateTime());
    setInterval();
    boot_time_unix=RomaniaTime.now();
    rtc.setTime(boot_time_unix); 
    WiFi.disconnect();

  } else {
    boot_time_unix += sleep_duration_s;  // Advance stored Unix time
    rtc.setTime(boot_time_unix);         // Update RTC with estimated time
    Serial.println("Woke from deep sleep: updating RTC time.");
  }

 // Print the time (formatted and raw)
  Serial.print("Boot count: ");
  Serial.println(boot_count);

  Serial.print("RTC Time: ");
  Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));  // Nice format

  Serial.print("Epoch Time: ");
  Serial.println(rtc.getEpoch());


  digitalWrite(5, LOW);
  BatteryLevel = analogRead(6);
  digitalWrite(5, HIGH);

  Serial.println();
  Serial.print("Battery adc: ");
  Serial.println(BatteryLevel);



  Serial.println("setup");

  
  SPI.begin(18, -1, 9, 19); // remap spi for EPD (swap pins), (sck, miso, mosi, ss)
 
  delay(100);
  if (boot_count = 1) {
    display.init(115200, true); // default 10ms reset pulse, e.g. for bare panels with DESPI-C02
  } else {
    display.init(115200, false);
  }
 
  Serial.println("setup done");

  // Your task
  // showTemp();
  showTemp();
  uint64_t end = esp_timer_get_time();
  
  // Go to deep sleep for 60 seconds
  Serial.println("Going to sleep for 60 seconds...");
  esp_sleep_enable_timer_wakeup(sleep_duration_us-end); // 60 seconds in microseconds
  Serial.flush(); // Wait for all serial output
  esp_deep_sleep_start();
}

void loop()
{
 
}
