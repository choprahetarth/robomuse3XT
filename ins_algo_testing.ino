#include <math.h>

/////////floating variables////////////
float accelX, velX, posX; 
float newAccelX, newVelX, newPosX;
float deltaAccelX, deltaVelX, deltaPosX;
float  velocityDecay, velocityNonDecay;
float positionDecay, positionNonDecay;
int time, time1, dt;
float count;
///////////////////////////////////////

//////// filter values ////////////////
float bandpassedAccelX, decayedAccelX, averageAccelX;
float newValue, alphaLow, alphaHigh, bandpassValue, filteredHigh, filteredLow, sensorReadings = 0, tValue = 0, decayFactor = 0, rValue=0, decayFactor1 =0;
///////////////////////////////////////

float randomNumber;
///////// filter codes ///////////////////////////////////////////////////////////////////
float bandpassFilter(float sensorReadings, float alphaLow, float alphaHigh){
      newValue = sensorReadings;
      filteredLow = (1-alphaLow)*filteredLow + alphaLow*newValue; ////ema lowpass filter
      filteredHigh = (1-alphaHigh)*filteredHigh + alphaHigh*newValue; /// ema highpass filter
      bandpassValue = filteredHigh - filteredLow;
      return bandpassValue;
  }

float decayFilter(float rawValue, float rValue, float tValue){
    decayFactor = pow(rValue,tValue-abs(rawValue));
    if (abs(rawValue)<tValue){
        rawValue = rawValue * decayFactor;
      }
    return rawValue;
  }

float averagingFilter(float unAveragedFilter, int rangeOfFilter){
   float filterSum=0, finalOutput=0;
   int count;
        for (count = 0 ; count < rangeOfFilter ; count++){
             filterSum = filterSum + unAveragedFilter;
            }
    finalOutput = (filterSum / rangeOfFilter);
    return finalOutput;
  }

///////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    Serial.begin(9600);
    time = millis();
    randomSeed(analogRead(0));
}

void loop() {
  accelerationCalculation();
  velocityDecay = velocityCalculation(decayedAccelX);
  velocityNonDecay = velocityCalculation(accelX);
  positionDecay = positionCalculation(velocityDecay);
  positionNonDecay = positionCalculation(velocityNonDecay);
}

void accelerationCalculation(){
      count = count + 0.1;

      ////////add more noise to the system///////
      randomNumber = random(-10,10);
      ///////////////////////////////////////////
      Serial.println(randomNumber);
      accelX = sin(count)+randomNumber;
      if (count > 20){accelX = 0; }
      averageAccelX = averagingFilter(accelX,10000);
      decayedAccelX = decayFilter(accelX, 0.01, 0.2);
  }

float velocityCalculation(float accelerationValue){
      time1 = millis();
      //dt = time1 - time;
      dt = 10;
      newAccelX = accelerationValue;
      deltaAccelX = (newAccelX - accelX);
      newVelX = velX + (0.5 * deltaAccelX *dt) + (accelX * dt);
      //// reset things here /////
      time = time1;
      accelX = newAccelX;
      velX = newVelX;
      return newVelX;
  }

float positionCalculation(float velocityValue){
      dt = 10;
      velocityValue = newVelX;
      deltaVelX = (newVelX - velX);
      newPosX = newPosX + (0.5 * deltaVelX * dt) + (velX *dt);
      return newPosX;
  }
