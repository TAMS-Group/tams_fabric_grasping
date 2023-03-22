#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

HX711 scale;

char   print_buffer[128];

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
}

void loop() {

  if (scale.is_ready()) {
    double loadcell_reading = (scale.read()+55050)/186.375465175663;  // force in mN
    int current_reading = map((double)analogRead(A0), 0, 1023, -5.0, 5.0) * 1000;  // current in mA
    //Serial.print("HX711 reading: ");
    sprintf( print_buffer, "L %d %d\n", (int)loadcell_reading, current_reading);
    Serial.print( print_buffer );
    Serial.flush();
  }
  delay(80);
  
}
