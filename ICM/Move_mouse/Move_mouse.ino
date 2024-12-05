#include <Mouse.h>

float x=100;
float y=100;
float wheel=0;

void setup() {
  // put your setup code here, to run once:

  Mouse.move(x,y,wheel);

}

void loop() {
  // put your main code here, to run repeatedly:
   // Mouse.move(x,y,wheel);
delay(2000);

}
