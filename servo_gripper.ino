#include <Servo.h>

Servo Gripper;

void setup() {
  Gripper.attach(53);
}

void loop() {
  //servo(3);
  delay(1000);
  servo(4);
  delay(1000);
  servo(5);
  delay(1000);

}

void servo(int pos){
  if (pos==6){
    Gripper.write(180);
    delay(200);
  }
  else if (pos==5){
    Gripper.write(105);
    delay(200);
  }
  else if (pos==4){
    Gripper.write(50);
    delay(200);
  }
  else if (pos==3){
    Gripper.write(0);
    delay(200);
  }
}