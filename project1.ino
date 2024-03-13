#include <Braccio.h>
#include<Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

const float L1 = 12.5;
const float L2 = 12.5;
const float L3 = 7.15;

void setup() {
  Serial.begin(5000);
  Braccio.begin();
  delay(1000);
}

void loop() {
  float x = -32.1, y = 0, theta = 180;
  inversekinematics(x, y, theta);
  delay(1000);
}

void inversekinematics(float x, float y, float theta) {

  float x1 = L3 * cos(radians(theta));
  float y1 = L3 * sin(radians(theta));
  float x3 = x - x1;
  float y3 = y + y1;
  float l = sqrt(pow(x3, 2) + pow(y3, 2));

  if ((L1 + L2) > l) {
    float thetaa = acos((pow(L1, 2) + pow(L2, 2) - pow(l, 2)) / (2 * L1 * L2));
    float thetab = acos((pow(L1, 2) + pow(l, 2) - pow(L2, 2)) / (2 * L1 * l));
    float a = degrees(thetaa);
    float B = degrees(thetab);

    float theta1 = degrees(atan2(y3, x3)) - B;
    float theta2 = 180 - a;
    float theta3 = degrees(atan2(y, x)) - theta1 - theta2;

    Braccio.ServoMovement(20, 0, int(theta1), 90 + int(theta2), 90 + int(theta3), 0, 73);

    Serial.println(theta1);
    Serial.println(theta2);
    Serial.println(theta3);
  }
}
