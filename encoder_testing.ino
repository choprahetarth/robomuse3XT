#include <Encoder.h>      // Header file for the Encoders

Encoder enCoder_1(20, 21); // Left hand side enc., +ve value means forward
Encoder enCoder_2(2, 3);  

float newLeftEncoderValue, leftEncoderIncrement, oldLeftEncoderValue, finalValueLeft;
float newRightEncoderValue, rightEncoderIncrement, oldRightEncoderValue, finalValueRight;
int pastTime, currentTime, dt;

void setup() {
    Serial.begin(115200); /// terminal window
    
}

void loop() {
    newLeftEncoderValue = double (enCoder_1.read());
    newRightEncoderValue = double (enCoder_2.read());
    leftEncoderIncrement = newLeftEncoderValue - oldLeftEncoderValue;
    rightEncoderIncrement = newRightEncoderValue - oldRightEncoderValue;
    finalValueLeft= finalValueLeft+ leftEncoderIncrement;
    finalValueRight= finalValueRight+ rightEncoderIncrement;
    oldLeftEncoderValue = newLeftEncoderValue; 
    oldRightEncoderValue = newRightEncoderValue; 
    Serial.print("left value");
    Serial.println(finalValueLeft);
    Serial.print("right value");
    Serial.println(finalValueRight);
    }
