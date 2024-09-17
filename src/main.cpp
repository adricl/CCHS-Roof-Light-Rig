#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>

#include <VL53L1X.h>
#include "SimpleKalmanFilter.h"

#define DIST_SENSOR_TO_LED  10


//LED
#define NUM_LEDS              170
#define NUM_LEDS_PER_METER    30
#define DATA_PIN_0            2
#define DATA_PIN_1            3
#define DATA_PIN_2            4
#define DATA_PIN_3            5
#define DATA_PIN_4            6
#define DATA_PIN_5            7

#define LED_TYPE            WS2811
#define COLOR_ORDER         GRB


CRGB leds[NUM_LEDS];
VL53L1X sensor;
int posLast = 0;
int noDataCount = 0;
int noDataCountThreshold = 20;
int prevPos = 0;
SimpleKalmanFilter simpleKalmanFilter(5, 5, 1);

int calcPosTimeOfFlight(){
  sensor.read();

  Serial.print("range: ");
  Serial.print(sensor.ranging_data.range_mm/10);
  Serial.print("\tstatus: ");
  Serial.print(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
  Serial.print("\tpeak signal: ");
  Serial.print(sensor.ranging_data.peak_signal_count_rate_MCPS);
  Serial.print("\tambient: ");
  Serial.println(sensor.ranging_data.ambient_count_rate_MCPS);

  //Convert to cm
  if (sensor.ranging_data.peak_signal_count_rate_MCPS > .4){
    uint16_t dist = sensor.ranging_data.range_mm / 10;
    float estimate = simpleKalmanFilter.updateEstimate(dist);
    int pos = (int)((estimate + DIST_SENSOR_TO_LED)  / (100 / NUM_LEDS_PER_METER));
    
    Serial.print("Pos: ");
    Serial.print(pos);
    Serial.print(" distance ");
    Serial.println(dist);
  
    noDataCount = 0;
    prevPos = pos;
    return pos;
  }
  else if (noDataCount < noDataCountThreshold) {
    noDataCount++;
    return prevPos;
  }

  return -1;
  
}

void setup() {
  Serial.begin(115200);

  //Setup Fast Led
  FastLED.addLeds<LED_TYPE, DATA_PIN_0, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, DATA_PIN_1, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, DATA_PIN_2, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, DATA_PIN_3, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, DATA_PIN_4, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, DATA_PIN_5, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  FastLED.setBrightness(255);
  FastLED.setDither( 255 );

  //Setup Wire for I2C for Time of Flight Sensor
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(150000);
  sensor.startContinuous(50);

  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

}
 
void loop() {
  // bool falseReading = false;
  int currPos = calcPosTimeOfFlight();
  int posDiffSize = abs(currPos - posLast) + 15;
  static byte colour;
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  colour += 1;

  int pos = int((posLast + currPos)/2);
  posLast = currPos;
  
  if (pos >= 0 && posDiffSize < 30){
    for(int i = 0; i < posDiffSize; i++){
      int ledPos = pos - posDiffSize/2 + i;
      if (ledPos < NUM_LEDS && ledPos >= 0){
        leds[ledPos] = CHSV(colour, 255, 255);    
      }
    }
  
  
    FastLED.show();

    for(int i = 0; i < posDiffSize; i++){
      int ledPos = pos - posDiffSize/2 + i;
      if (ledPos < NUM_LEDS && ledPos >= 0){
        leds[ledPos] = CRGB::Black; 
      }
    }
  }
  else {
    
    pride();
    FastLED.show();
    
  }
}

void pride() 
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;
 
  uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);
 
  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);
  
  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5,9);
  uint16_t brightnesstheta16 = sPseudotime;
  
  for( uint16_t i = 0 ; i < NUM_LEDS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;
 
    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;
 
    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);
    
    CRGB newcolor = CHSV( hue8, sat8, bri8);
    
    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS-1) - pixelnumber;
    
    nblend( leds[pixelnumber], newcolor, 64);
  }
}
