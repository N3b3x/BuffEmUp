#include <Sparki.h>

#define DEBUG         0     // Set to 1 to allow certain debug outputs

const int threshold = 500;  // IR reading threshold to detect whether there's a black line under the sensor

int state       = 0;
int cm_distance = 0;
int line_left   = 0;
int lineCenter  = 0;
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

void loop() {
  // Display the state on the LCD screen
  sparki.clearLCD();
  sparki.print("STATExx: ");
  sparki.println(state);

  // State Machine
  switch (state)
  {
    case 0:
      state = StateZero();
      break;
    case 1:
      state = stateOne();
      break;
    case 2:
      state = stateTwo();
      break;
    case 3:
      state = stateThree();
      break;
    case 4:
      state = stateFour();
      break;
    case 5:
      state = stateFive();
      break;
    case 6:
      state = stateSix();
      break;
    case 7:
      sparki.println("Task Done");
      break;
  }

  sparki.updateLCD();
  delay(100); // Only run controller at 10Hz
}

//============================================================================//
// Rotate until it detects an object within 30cm
//============================================================================//
int StateZero()
{
  int cm = sparki.ping();
#if DEBUG
  sparki.print(cm);
#endif

  if (cm > 30) // make sure its not too close or too far or if the distance measured is greater than 30 centimeters
  {
    // turn the light red
    sparki.RGB(RGB_RED);

    // Rotate around it's center
    //sparki.moveLeft();
    sparki.moveRight(15);
  }
  else {
    sparki.moveStop();
    return 1;
  }
  return 0;
}

//============================================================================//
// Drive the robot within 7cm of the detected object
//============================================================================//
int stateOne()
{
  int cm = sparki.ping();
  sparki.print(cm);
  sparki.updateLCD();
  if (cm > 7) {           // make sure its not too close or too far or if the distance measured is greater than 7 centimeters
    sparki.RGB(RGB_BLUE); // Turn the light blue
    sparki.moveForward(); // Move Forward
  }
  else {
    sparki.moveStop();
    return 2;
  }
  return 1;
}

// Capture the object
int stateTwo() {
  sparki.print("made to close");
  sparki.updateLCD();
  sparki.gripperClose(5); // close the robot's gripper
  delay(5000);
  //sparki.gripperClose();
  sparki.print("made to after");
  sparki.updateLCD();
  delay(5000);
  return 3;
}

//============================================================================//
// Turn Robot 180 degrees
//============================================================================//
int stateThree() {
  sparki.updateLCD();
  sparki.print("made to turnind");
  //  sparki.updateLCD();
  delay(2000);
  sparki.moveRight(90);
  sparki.RGB(RGB_RED);

  sparki.moveLeft(-90);

#if (DEBUG)
  delay(3000);
  sparki.print("Fully tunred");
  sparki.updateLCD();
  delay(3000);
  sparki.clearLCD();
#endif

  delay(3000);
  return 4;
}

//============================================================================//
// Drive it till it detects a line
//============================================================================//
int stateFour() {
  line_left   = sparki.lineLeft();    // measure the left IR sensor
  line_right  = sparki.lineRight();   // measure the right IR sensor
  line_center = sparki.lineCenter();
  //sparki.print("made to 4");
  //sparki.updateLCD();
  //delay(3000);
  if (( line_center < threshold) || (line_left < threshold) || (line_right < threshold)) {
    sparki.moveStop();
    return 5;
  }
  else {
    //sparki.print("moving Forward");
    //sparki.updateLCD();
    sparki.moveForward(10);
    delay(3000);
  }
  return 4;
}

//============================================================================//
// Follow line until it reaches the "start" marker, then stop
//============================================================================//
int stateFive() {
  bool  Looper = true;
  while (Looper == true)
  {
    line_left   = sparki.lineLeft();    // measure the left IR sensor
    line_right  = sparki.lineRight();   // measure the right IR sensor
    line_center = sparki.lineCenter();  // measure the center IR sensor

    if ((line_center < threshold) && (line_left < threshold) && (line_right < threshold)) {
      sparki.moveStop();
      Looper = false;
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
    delay(100);
  }
  return 6;
}

//============================================================================//
// Beeps, and drops the carried object
//============================================================================//
int stateSix() {
  sparki.beep();            // Beep the robot
  sparki.gripperOpen();     // open the robot's gripper
  delay(1000);              // for 1 second (1000 milliseconds)
  sparki.gripperClose(5);   // close the robot's gripper
  return 7;
}
