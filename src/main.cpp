#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>

#include <VL53L1X.h>
#include "SimpleKalmanFilter.h"

#define DIST_SENSOR_TO_LED  25

//LED
#define NUM_LEDS              171
#define NUM_LEDS_PER_METER    30
#define DATA_PIN_0            18
#define DATA_PIN_1            5
#define DATA_PIN_2            4
#define DATA_PIN_3            2
#define DATA_PIN_4            15
#define DATA_PIN_5            13

#define LED_TYPE            WS2812
#define COLOR_ORDER         GRB

#define LED_STRIPS          6

CRGB leds[LED_STRIPS][NUM_LEDS];
VL53L1X sensor;
int posLast = 0;
int falsePosCount = 0;
SimpleKalmanFilter simpleKalmanFilter(10, 10, .1);

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
  uint16_t dist = sensor.ranging_data.range_mm / 10;
  float estimate = simpleKalmanFilter.updateEstimate(dist);
  int pos = (int)((estimate + DIST_SENSOR_TO_LED)  / (100 / NUM_LEDS_PER_METER));
  Serial.print("Pos: ");
  Serial.print(pos);
  Serial.print(" distance ");
  Serial.println(dist);
  return pos;
}

void setup() {
  Serial.begin(115200);

  //Setup Fast Led
  FastLED.addLeds<LED_TYPE, DATA_PIN_0, COLOR_ORDER>(leds[0], NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, DATA_PIN_1, COLOR_ORDER>(leds[1], NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, DATA_PIN_2, COLOR_ORDER>(leds[2], NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, DATA_PIN_3, COLOR_ORDER>(leds[3], NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, DATA_PIN_4, COLOR_ORDER>(leds[4], NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, DATA_PIN_5, COLOR_ORDER>(leds[5], NUM_LEDS).setCorrection(TypicalLEDStrip);

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
}
 
void loop() {
  bool falseReading = false;
  int currPos = calcPosTimeOfFlight();
  int posDiffSize = abs(currPos - posLast) + 5;
  static byte colour;
  colour += 1;

  int pos = int((posLast + currPos)/2);
  posLast = currPos;
  
  if (pos > 100){
    falsePosCount ++;
  }
  else {
    falsePosCount --;
  }
  if (falsePosCount > 15){
    falseReading = true;
    falsePosCount = 15;
  }
  

  if (pos >= 0 && !falseReading){
    for(int i = 0; i < posDiffSize; i++){
      int ledPos = pos + i;
      if (ledPos < NUM_LEDS && ledPos >= 0){
        for(int j = 0; j < LED_STRIPS; j++){
          leds[j][ledPos] = CHSV(colour, 255, 255);    
        }
        
      }
    }
  
  
    FastLED.show();

    for(int i = 0; i < posDiffSize; i++){
      int ledPos = pos + i;
      if (ledPos < NUM_LEDS && ledPos >= 0){
        for(int j = 0; j < LED_STRIPS; j++){
          leds[j][ledPos] = CRGB::Black; 
        }
      }
    }
  }
  else {
    for(int i = 0; i < LED_STRIPS; i++){
      fill_solid(leds[i], NUM_LEDS, CRGB::Black);
      FastLED.show();
    }
  }
}