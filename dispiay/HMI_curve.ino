#include <Arduino.h>
#include "hmi.h"

HMI hmi;
uint32_t count = 0;
uint32_t cycle = 20;
float amp = 0.0;
bool flag = false;
const int size = 250;
uint8_t data[size];

float curve_gene() {
  float val = float(count % cycle) / (float)cycle;
  if(val > 0.5) val = (1.0 - val) * 2.0;
  else val = val * 2.0;
  count++;
  //flag = true;
  return val;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  hmi.init(115200);
  hmi.curve_init(1000, 1, 0);
  hmi.curve_clear();
}

void loop() {
  for(int i = 0; i < size; i++) {
    //float angle = ((float)(i % cycle) / (float)cycle) * 2.0 * 3.1416;
    //float val = sin(angle) * 125.0 + 125.0;
    data[i] = i;
  }
  // hmi.curve_clear();
  for(int i = 0; i < 4; i++)
    hmi.curve_add_point(data, size);

  //hmi.curve_add_point(10, (uint8_t)(curve_gene() * 250));
  delay(1000);
}