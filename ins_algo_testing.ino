#include <math.h>

float accelX, velX, posX; 
float newAccelX, newVelX, newPosX;
float deltaAccelX, deltaVelX, deltaPosX;
int time, time1, dt;
float count;

void setup() {
    Serial.begin(9600);
    time = millis();
}

void loop() {
  accelerationCalculation();
  velocityCalculation();

}

void accelerationCalculation(){
      count = count + 0.1;
      accelX = sin(count);
      if (count > 20){accelX = 0; }
  
  }

void velocityCalculation(){
      time1 = millis();
      dt = time1 - time;
      deltaAccelX = (newAccelX - accelX);
      newVelX = velX + (0.5 * deltaAccelX *dt) + (accelX * dt);
      deltaVelX = (newVelX - velX);
      newPosX = newPosX + (0.5 * deltaVelX * dt) + (velX *dt);
      Serial.print(",");
      Serial.println(newPosX / 1000);
      Serial.print(dt);

      //// reset things here /////
      time = time1;
      accelX = newAccelX;
      velX = newVelX;
      
  }
