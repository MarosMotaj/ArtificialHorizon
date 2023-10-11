#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>
#include <BluetoothSerial.h>
#include <Preferences.h> // EEPROM

// For S6D02A1 based TFT displays
//#include <TFT_S6D02A1.h>         // Graphics and font library for S6D02A1 driver chip
//TFT_S6D02A1 tft = TFT_S6D02A1(); // Invoke library, pins defined in User_Setup.h

// For ST7735 based TFT displays
//#include <TFT_ST7735.h>          // Graphics and font library for ST7735 driver chip and Arduino ARV chip
//TFT_ST7735 tft = TFT_ST7735();   // Invoke library, pins defined in User_Setup.h

// For ST7735 based TFT displays
#include <TFT_eSPI.h>              // Graphics and font library for ST7735 driver chip and ESP32 ARM chip
TFT_eSPI tft = TFT_eSPI();  

// For ILI9341 based TFT displays (note sketch is currently setup for a 160 x 128 display)
//#include <TFT_ILI9341.h>         // Graphics and font library for ILI9341 driver chip
//TFT_ILI9341 tft = TFT_ILI9341(); // Invoke library, pins defined in User_Setup.h

#define REDRAW_DELAY 16 // minimum delay in milliseconds between display updates
#define HOR 172    // Horizon vector line length
#define BROWN      0x5140 //0x5960
#define SKY_BLUE   0x02B5 //0x0318 //0x039B //0x34BF
#define DARK_RED   0x8000
#define DARK_GREY  0x39C7
#define XC 64 // x coord of centre of horizon
#define YC 80 // y coord of centre of horizon
#define DEG2RAD 0.0174532925
#define BAUD_RATE 9600  
#define DEVICE_NAME "ARTIFICIAL_HORIZON"
#define GROUND "ground"
#define EEPROM_NAMESPACE "eeprom-vars"

/*****************************************Variables*************************************/
int last_roll = 0; 
int last_pitch = 0;
int ground_zero = 0;
int *p_ground_zero = &ground_zero;
unsigned long redrawTime = 0;

// Variables for test only
int test_roll = 0;
int delta = 0;

// MPU6050
sensors_event_t a, g, temp;
int roll = 0;
int pitch = 0;

/*******************************************Objects*************************************/
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;  // the TinyGPS++ object
BluetoothSerial SerialBT;
Preferences EEPROM; // EEPROM memory object

/***************************************Function prototypes*****************************/
void set_ground_zero();
void drawHorizon(int roll, int pitch);
void drawInfo();
void updateHorizon(int roll, int pitch);
void testRoll();


// #########################################################################
// Setup, runs once on boot up
// #########################################################################

void setup(void) {
  Serial.begin(BAUD_RATE);
  Serial2.begin(BAUD_RATE); // GPS
  EEPROM.begin(EEPROM_NAMESPACE, false);

  // Set ground level variable data from EEPROM memory
  *p_ground_zero = EEPROM.getInt(GROUND, 0);

  // Initialize Bluetooth
  if (!SerialBT.begin(DEVICE_NAME)) {
    Serial.println("Failed to initialize BluetoothSerial.");
  }

  // MPU6050 
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  // For display
  tft.begin();
  tft.setRotation(0);
  tft.fillRect(0,  0, 128, 80, SKY_BLUE);
  tft.fillRect(0, 80, 128, 80, BROWN);

  // Draw the horizon graphic
  drawHorizon(0, 0);
  drawInfo();
  delay(2000); // Wait to permit visual check

  // Clear LCD from garbage on the first start up
  testRoll();

}


// #########################################################################
// Main loop, keeps looping around
// #########################################################################

void loop() {
  
  set_ground_zero();

  /* Get new sensor events with the readings */
  mpu.getEvent(&a, &g, &temp);
  
  // Refresh the display at regular intervals
  if (millis() > redrawTime) {
    
    redrawTime = millis() + REDRAW_DELAY;

    roll = ((a.gyro.x * 9) * -1);
    pitch = (a.gyro.y * 9);

    updateHorizon(roll, pitch);
    
  }

  delay(100);
}

// #########################################################################
// Method is used for level ground on airfield
// #########################################################################

void set_ground_zero() {
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil(';');
    //Serial.printf("Command: %s\n", command); //For debug

    if (!command.isEmpty()) {
      if (command.startsWith(GROUND)) {
        EEPROM.putInt(GROUND, (command.substring(7)).toInt());
        *p_ground_zero = EEPROM.getInt(GROUND, 0);
        SerialBT.println("Saved!");
        // Clear LCD from garbage
        testRoll();
      } 
      else if (command.equals("exit")) {
        SerialBT.disconnect();
      }
      else {
        SerialBT.println("Enter next command");
      }
    }
  }
}

// #########################################################################
// Update the horizon with a new roll (angle in range -180 to +180)
// #########################################################################

void updateHorizon(int roll, int pitch)
{
  bool draw = 1;
  int delta_pitch = 0;
  int pitch_error = 0;
  int delta_roll  = 0;
  while ((last_pitch != pitch) || (last_roll != roll))
  {
    delta_pitch = 0;
    delta_roll  = 0;

    if (last_pitch < pitch) {
      delta_pitch = 1;
      pitch_error = pitch - last_pitch;
    }

    if (last_pitch > pitch) {
      delta_pitch = -1;
      pitch_error = last_pitch - pitch;
    }

    if (last_roll < roll) delta_roll  = 1;
    if (last_roll > roll) delta_roll  = -1;

    if (delta_roll == 0) {
      if (pitch_error > 1) delta_pitch *= 2;
    }

    drawHorizon(last_roll + delta_roll, last_pitch + delta_pitch);
    drawInfo();
  }
}

// #########################################################################
// Draw the horizon with a new roll (angle in range -180 to +180)
// #########################################################################

void drawHorizon(int roll, int pitch)
{
  // Calculate coordinates for line start
  float sx = cos(roll * DEG2RAD);
  float sy = sin(roll * DEG2RAD);

  int16_t x0 = sx * HOR;
  int16_t y0 = sy * HOR;
  int16_t xd = 0;
  int16_t yd = 1;
  int16_t xdn  = 0;
  int16_t ydn = 0;

  if (roll > 45 && roll <  135) {
    xd = -1;
    yd =  0;
  }
  if (roll >=  135)             {
    xd =  0;
    yd = -1;
  }
  if (roll < -45 && roll > -135) {
    xd =  1;
    yd =  0;
  }
  if (roll <= -135)             {
    xd =  0;
    yd = -1;
  }

  if ((roll != last_roll) && ((abs(roll) > 35)  || (pitch != last_pitch)))
  {
    xdn = 4 * xd;
    ydn = 4 * yd;
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);
    xdn = 3 * xd;
    ydn = 3 * yd;
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);
  }
  xdn = 2 * xd;
  ydn = 2 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);
  
  tft.drawLine(XC - x0 - xd, YC - y0 - yd - pitch, XC + x0 - xd, YC + y0 - yd - pitch, SKY_BLUE);
  tft.drawLine(XC - x0 + xd, YC - y0 + yd - pitch, XC + x0 + xd, YC + y0 + yd - pitch, BROWN);
  
  tft.drawLine(XC - x0, YC - y0 - pitch,   XC + x0, YC + y0 - pitch,   TFT_YELLOW);

  last_roll = roll;
  last_pitch = pitch;
}

// #########################################################################
// Draw the information
// #########################################################################

void drawInfo(void)
{
  // GPS
  if (Serial2.available() > 0) {
    gps.encode(Serial2.read());
    tft.drawString("SPD", 110, 70);
    tft.drawNumber(gps.speed.kmph(), 110, 80, 1);
    tft.drawString("Km/h", 110, 90, 1);
    tft.drawString("ALT", 20, 70);
    tft.drawNumber(gps.altitude.meters() - ground_zero, 20, 80, 1);
    tft.drawString("mtr", 20, 90, 1);
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

  
  // Update things near middle of screen first (most likely to get obscured)

  // Level wings graphic
  tft.fillRect(64 - 1, 80 - 1, 3, 3, TFT_RED);
  tft.drawFastHLine(64 - 30,   80, 24, TFT_RED);
  tft.drawFastHLine(64 + 30 - 24, 80, 24, TFT_RED);
  tft.drawFastVLine(64 - 30 + 24, 80, 3, TFT_RED);
  tft.drawFastVLine(64 + 30 - 24, 80, 3, TFT_RED);

  // Pitch scale
  tft.drawFastHLine(64 - 12,   80 - 60, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 - 50, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 - 40, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 - 30, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 - 20, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 - 10, 12, TFT_WHITE);

  tft.drawFastHLine(64 -  6,   80 + 10, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 + 20, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 + 30, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 + 40, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 + 50, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 + 60, 24, TFT_WHITE);

  // Pitch scale values
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(64 - 12 - 13, 80 - 20 - 3);
  tft.print("10");
  tft.setCursor(64 + 12 + 1, 80 - 20 - 3);
  tft.print("10");
  tft.setCursor(64 - 12 - 13, 80 + 20 - 3);
  tft.print("10");
  tft.setCursor(64 + 12 + 1, 80 + 20 - 3);
  tft.print("10");

  tft.setCursor(64 - 12 - 13, 80 - 40 - 3);
  tft.print("20");
  tft.setCursor(64 + 12 + 1, 80 - 40 - 3);
  tft.print("20");
  tft.setCursor(64 - 12 - 13, 80 + 40 - 3);
  tft.print("20");
  tft.setCursor(64 + 12 + 1, 80 + 40 - 3);
  tft.print("20");

  tft.setCursor(64 - 12 - 13, 80 - 60 - 3);
  tft.print("30");
  tft.setCursor(64 + 12 + 1, 80 - 60 - 3);
  tft.print("30");
  tft.setCursor(64 - 12 - 13, 80 + 60 - 3);
  tft.print("30");
  tft.setCursor(64 + 12 + 1, 80 + 60 - 3);
  tft.print("30");

  // Display justified roll value near bottom of screen
  tft.setTextColor(TFT_YELLOW, BROWN); // Text with background
  tft.setTextDatum(MC_DATUM);            // Centre middle justified
  tft.drawString("Temp:", 110, 10);
  tft.drawNumber(temp.temperature, 110, 20, 1);

}

// #########################################################################
// Function to generate roll angles for testing only
// #########################################################################

int rollGenerator(int maxroll)
{
  // Synthesize a smooth +/- 50 degree roll value for testing
  delta++; if (delta >= 360) test_roll = 0;
  test_roll = (maxroll + 1) * sin((delta) * DEG2RAD);

  // Clip value so we hold roll near peak
  if (test_roll >  maxroll) test_roll =  maxroll;
  if (test_roll < -maxroll) test_roll = -maxroll;

  return test_roll;
}

// #########################################################################
// Function to generate roll angles for testing only
// #########################################################################

void testRoll(void)
{
  for (int a = 0; a < 360; a++) {
    //delay(REDRAW_DELAY / 2);
    updateHorizon(rollGenerator(180), 0);
  }
}
