#include <Sparki.h>

const int threshold = 700; // IR reading threshold to detect whether there's a black line under the sensor

int state       = 0;
int cm_distance = 0;
int line_left   = 0;
int line_center = 0;
int line_right  = 0;

void setup() {
  sparki.RGB(RGB_RED);        // Turn on the red LED
  sparki.servo(SERVO_CENTER); // Center the ultrasonic sensor
  delay(1000);                // Give the motor time to turn
  sparki.gripperOpen();       // Open the gripper
  delay(5000);                // Give the motor time to open the griper
  sparki.gripperStop();       // 5 seconds should be long enough
  sparki.RGB(RGB_GREEN);      // Change LED to green so we know the robot's setup is done!
}

void readSensors() {
  cm_distance = sparki.ping();        // read the distance
  line_left   = sparki.lineLeft();    // measure the left IR sensor
  line_right  = sparki.lineRight();   // measure the right IR sensor
  line_center = sparki.lineCenter();  // measure the center IR sensor
}

void loop() {
  // put your main code here, to run repeatedly:
  readSensors(); // Read sensors once per loop() call

  sparki.clearLCD();
  sparki.print("STATExx: ");
  sparki.println(state);

  // Your state machine code goes here
  switch (state)
  {
    case 0:
      state = StateZero();
      break;
    case 1:
      state = StateOne();
      break;
    case 2:
      state = StateTwo();
      break;
    case 3:
      state = StateThree();
      break;
    case 4:
      state = StateFour();
      break;
    case 5:
      state = StateFive();
      break;
    case 6:
      state = StateSix();
      break;
    case 7:
      sparki.println("Task Done")
      break;
  }

  sparki.updateLCD();
  delay(100); // Only run controller at 10Hz
}

// Rotate until it detects an object within 30cm
int StateZero()
{
  if (cm != -1 || cm > 30) // make sure its not too close or too far or if the distance measured is greater than 30 centimeters
  {
    // turn the light red
    sparki.RGB(RGB_RED);

    // Rotate around it's center
    sparki.moveLeft();
    sparki.moveRight();
  }
  else {
    sparki.moveStop();
    //delay(2000);
    return 1;
  }
  return 0;
}

// Drive the robot within 7cm of the detected object
int stateOne()
{
  if (cm != -1 || cm > 7) // make sure its not too close or too far or if the distance measured is greater than 7 centimeters
  {
    // Turn the light blue
    sparki.RGB(RGB_BLUE);

    // Move Forward
    sparki.moveForward();
  }
  else if (cm == -1) {
    sparki.moveStop();
    return 0;
  }
  else {
    sparki.moveStop();
    return 2;
  }

  return 1;
}

// Capture the object
int stateTwo() {
  sparki.gripperClose(); // close the robot's gripper
  delay(1000);
  sparki.gripperClose();
  return 3;
}

// Turn Robot 180 degrees
int stateThree() {
  sparki.moveRight(180);
  sparki.moveLeft(-180);
  delay(3000);
  return 3;
}

// Drive it till it detects a line
int stateFour() {
  if ((lineCenter < threshold) || (line_left < threshold) || (line_right < threshold)) {
    sparki.moveStop();
    return 5;
  }
  else{
    sparki.moveForward();
  }
  return 4;
}

// Follow line until it reaches the "start" marker, then stop
int stateFive() {

  if ((lineCenter < threshold) && (line_left < threshold) && (line_right < threshold)) {
    sparki.moveStop();
    return 6;
  }
  else if ( line_center < threshold ) { // if line is below center line sensor
    sparki.moveForward(); // move forward
  }
  else {
    if ( line_left < threshold ) // if line is below left line sensor
    {
      sparki.moveLeft(); // turn left
    }

    if ( line_right < threshold ) // if line is below right line sensor
    {
      sparki.moveRight(); // turn right
    }
  }
  return 5;
}
// Beeps, and drops the carried object
int stateSix() {
  sparki.beep();          // Beep the robot
  sparki.gripperOpen();   // open the robot's gripper
  delay(1000);            // for 1 second (1000 milliseconds)
  sparki.gripperClose();  // close the robot's gripper
  return 7;
}
