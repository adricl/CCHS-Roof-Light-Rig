#include <Arduino.h>
#include <Wire.h>
#include <NewPing.h>
#include <FastLED.h>

#include <VL53L1X.h>


//UltraSonics
#define TRIGGER_PIN         2
#define ECHO_PIN            4
#define MAX_DISTANCE        200

#define DIST_SENSOR_TO_LED  10

//LED
#define NUM_LEDS            171
#define NUM_LEDS_PER_METER  40
#define DATA_PIN            5
#define LED_TYPE            WS2812
#define COLOR_ORDER         GRB

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
CRGB leds[NUM_LEDS];

VL53L1X sensor;

int calcPosUltrasonic(){
  long dist = sonar.ping_cm();
  Serial.print("Distance: ");
  Serial.print(dist);
  Serial.println("cm ");
  if (dist > DIST_SENSOR_TO_LED && dist < MAX_DISTANCE) {
      int ledCount = (int)((dist - DIST_SENSOR_TO_LED)  / (100 / NUM_LEDS_PER_METER));
      Serial.print("Pos: ");
      Serial.print(NUM_LEDS - ledCount);
      Serial.print(" ledCount: ");
      Serial.println(ledCount);
      return NUM_LEDS - ledCount;

  }
  else {
      return -1;
  }
}

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
  int pos = (int)(dist  / (100 / NUM_LEDS_PER_METER));
  Serial.print("Pos: ");
  Serial.print(pos);
  Serial.print(" distance ");
  Serial.println(dist);
  return pos;
}

void setup() {
  Serial.begin(115200);

  //Setup Fast Led
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
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
  //calcPosUltraSonic();
  int pos = calcPosTimeOfFlight();
  if (pos > 0){
    for(int i = 0; i < 5; i++){
      if (pos + 1 <= NUM_LEDS){
        leds[pos + i] = CHSV(pos, 255, 255);    
      }
    }
  }
  
  FastLED.show();
  delay(100);

    for(int i = 0; i < 5; i++){
      if (pos + 1 <= NUM_LEDS){
        leds[pos + i] = CRGB::Black;    
      }
    }
}