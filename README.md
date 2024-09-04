# CCHS-Roof-Light-Rig
CCHS Roof Light Rig to detect people

This consists of 6 strips of 170 leds attached to the roof of the space. 
This is all controlled by an Ardunio UNO. 
We have also attached a VL53L1X distance sensor https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html

## Wiring the VL53L1X distance sensor
To connect this to an Arduino Uno we can use both SDA and SCL or A4(SDA) and A5(SCL).
This also required ground and 5v/3.3v to be attached. The sensor has a power regulator on it so it can accept many voltages.

For some reason the SDA and SCL can be used when the sensor is powered by 5v, but the A4 and A5 data lines must use 3.3v to power the sensor. I am not sure why.

