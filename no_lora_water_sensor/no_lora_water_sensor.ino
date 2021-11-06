#include <Arduino.h>

#include <LoRaWan-RAK4630.h> //Click here to get the library: http://librarymanager/All#SX126x

#include <SPI.h>

#define SENSOR_DEPTH 1.0

// Check if the board has an LED port defined
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

int get_depths(void)
{
  int i;

  int sensor_pin = WB_A0;   // select the input pin for the potentiometer
  int mcu_ain_raw = 0; // variable to store the value coming from the sensor

  int depths; // variable to store the value of oil depths
  int average_raw;
  float voltage_ain, factor;

  for (i = 0; i < 5; i++)
  {
    mcu_ain_raw += analogRead(sensor_pin);
  }
  average_raw = mcu_ain_raw / i;
  voltage_ain = average_raw * (3.0/1024.0);
  

  factor = (voltage_ain-0.5)/2.0;
  depths = factor*SENSOR_DEPTH*1000;
  
  Serial.printf("read value = %d = %f v = %d mm\n" , average_raw, voltage_ain, depths);
  
  return depths;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  /*
     WisBLOCK 5811 Power On
  */
  //pinMode(WB_IO1, OUTPUT);
  //digitalWrite(WB_IO1, HIGH);

  pinMode(WB_A0, INPUT_PULLDOWN);
  analogReference(AR_INTERNAL_3_0);
  //analogReference(AR_INTERNAL);
  analogOversampling(128);

  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  get_depths();
  delay(5000);
}
