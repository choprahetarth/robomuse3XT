String commandFromSerial;

void displayControls() {
  Serial.println("Controls are as -");
  Serial.println("\t f = forward,\n\t b = back,\n\t r = right,\n\t l = left,");
  Serial.println("\t inc = increase speed,\n\t dec = decrease speed,\n\t s = stop,");
  Serial.println("\t exit = exit manual mode.");
}

void driveThru() {
  // continuous straight movement forward or backward, using encoder feedback
  // to correct motor speeds, calculated using PID
  if (motionType == "f" || motionType == "b") {
    //    obstacleStop();   // Check for Obstacle in path, if Yes.. STOP
    a = 0;
    odometryCalc();   // Calculate positional error value
    input = feedbackVariable; // Error value passed to PID input
    PID_L.Compute(); PID_R.Compute();// PID computes for Controller outputs
    leftMotorSpeed += outputL;
    rightMotorSpeed -= outputR;
    saberTooth.motor(1, leftMotorSpeed);
    saberTooth.motor(2, rightMotorSpeed);
    delay(50);
    //Serial.println(outputR);
    //Serial.println(rightMotorSpeed);
    //Serial.println(outputL);
  }
  else { // continue with given speeds
    saberTooth.motor(1, leftMotorSpeed);
    saberTooth.motor(2, rightMotorSpeed);
    delay(50);
  }
}

void manualMode() {
  displayControls();
  while (true) { // Repeat until broken using interrupt
    while (!Serial.available()) {
      driveThru();
    }
    commandFromSerial = Serial.readString();

    if (commandFromSerial == "f") {
      Serial.println("Moving Forward.");
      resetCoordinates();
      leftMotorSpeed = 25;
      rightMotorSpeed = 25;
      motionType = "f";
    }
    else if (commandFromSerial == "b") {
      Serial.println("Moving Backward.");
      resetCoordinates();
      leftMotorSpeed = -25;
      rightMotorSpeed = -25;
      motionType = "b";
    }
    else if (commandFromSerial == "r") {
      Serial.println("Moving Right.");
      leftMotorSpeed = 25;
      rightMotorSpeed = -25;
      motionType = "r";
    }
    else if (commandFromSerial == "l") {
      Serial.println("Moving Left.");
      leftMotorSpeed = -25;
      rightMotorSpeed = 25;
      motionType = "l";
    }
    else if (commandFromSerial == "inc") {
      if (motionType == "f") {
        leftMotorSpeed += 5;
        rightMotorSpeed += 5;
      }
      else if (motionType == "l") {
        leftMotorSpeed -= 5;
        rightMotorSpeed += 5;
      }
      else if (motionType == "r") {
        leftMotorSpeed += 5;
        rightMotorSpeed -= 5;
      }
      else {
        leftMotorSpeed -= 5;
        rightMotorSpeed -= 5;
      }
      Serial.println("Speed Increased.");
    }
    else if (commandFromSerial == "dec") { // will switch direction of motion if n*5 > initial
      if (motionType == "f") {
        leftMotorSpeed -= 5;
        rightMotorSpeed -= 5;
      }
      else if (motionType == "l") {
        leftMotorSpeed += 5;
        rightMotorSpeed -= 5;
      }
      else if (motionType == "r") {
        leftMotorSpeed -= 5;
        rightMotorSpeed += 5;
      }
      else {
        leftMotorSpeed += 5;
        rightMotorSpeed += 5;
      }
      Serial.println("Speed Decreased");
    }
    else if (commandFromSerial == "s") {
      saberTooth.stop();
      leftMotorSpeed = 0;
      rightMotorSpeed = 0;
      Serial.println("Stopped.");
    }
    else if (commandFromSerial == "exit") {
      Serial.print("Exiting Manual mode..\t");
      modeSelected = "none"; // Switch to invalid mode
      Serial.println("Done.");
      saberTooth.stop();
      break;  // break out of while
    }
    else {
      Serial.println("Invalid Operand !");
      displayControls();
    }
    driveThru();  // not required as well, i guess.. anyway we'll be repeatedly calling it untill next instruction from Serial.
  }
  return; // not required, i guess
}
