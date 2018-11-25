#include <Servo.h>

  Servo motor;

void setup() {
  // put your setup code here, to run once:

  motor.attach(6);
  motor.writeMicroseconds(0);

  Serial.begin(115200);
  Serial.println("Calibration ready: 0 min throtle / 1 middle throtle/ 2 max throtle");
  
}

void loop() {
  
      if (Serial.available() > 0) {
      int response = Serial.read();
      if(response == 49){
        Serial.println("middle throtle");
        motor.writeMicroseconds(1500);
      }
      if(response == 48){        
        Serial.println("min throtle");
        motor.writeMicroseconds(1000);
      }     
      if(response == 50){
        Serial.println("max throtle");
        motor.writeMicroseconds(2000);
      }    
    }

}
