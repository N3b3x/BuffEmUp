// Speed of wheel = 0.02857143 m/s
// 30 cm in 10.5 second

#include <Sparki.h> // include the sparki library
//========================================================================//
// CONSTANTS
//========================================================================//
const int threshold     = 500;            // IR reading threshold to detect whether there's a black line under the sensor

const double wheelSpeed = 2.857;          // Speed of wheel in [cm/s]
const double w2w_dist   = 8.35;           // Wheel to Wheel distance [cm]
const double wheel_rad  = 2.5;            // Wheel Radius [cm]

const double CYCLE_TIME = 0.100;

//========================================================================//
// VARIABLES
//========================================================================//
int startTime = 0;
int stopTime  = 0;
int loopTime  = 0;

int line_left   = 0;
int line_center = 0;
int line_right  = 0;

double pose[3] = {0.0, 0.0, 0.0};
double theta   = 0;

double speed_r = 0;
double speed_l = 0;

double x_dot     = 0;
double y_dot     = 0;
double theta_dot = 0;

//========================================================================//
// FUNCTIONS
//========================================================================//
void updateSensors() {
  line_left   = sparki.lineLeft();    // measure the left IR sensor
  line_right  = sparki.lineRight();   // measure the right IR sensor
  line_center = sparki.lineCenter();  // measure the center IR sensor
}

void displayOdometry() {
  sparki.clearLCD();
  sparki.print("Loop Time = "); // show right line sensor on screen
  sparki.println(loopTime);
  sparki.print("X = ");
  sparki.println(pose[0]);
  sparki.print("Y = ");
  sparki.println(pose[1]);
  sparki.print("Angle = ");
  sparki.println(pose[2] * 180 / 3.14);

  sparki.print("x_dot = ");
  sparki.println(x_dot);
  sparki.print("y_dot = ");
  sparki.println(y_dot);
  sparki.print("theta_dot = ");
  sparki.println(theta_dot);
  sparki.updateLCD();                 // display all of the information written to the screen
}

void updateOdometry() {
  x_dot     = ((speed_r + speed_l) / 2) * cos(pose[2]);
  y_dot     = ((speed_r + speed_l) / 2) * sin(pose[2]);
  theta_dot = ((speed_r - speed_l) / (w2w_dist));

  pose[0] = pose[0] + x_dot * CYCLE_TIME;
  pose[1] = pose[1] + y_dot * CYCLE_TIME;
  pose[2] = pose[2] + theta_dot * CYCLE_TIME;
}

//========================================================================//
// VOID SETUP
//========================================================================//
void setup() {
}

//========================================================================//
// VOID LOOP
//========================================================================//
void loop() {
  startTime = millis();
  updateSensors();

  if ((line_center < threshold) && (line_left < threshold) && (line_right < threshold)) {
    sparki.moveStop();
    speed_r = 0;
    speed_l = 0;
  }
  else if ( line_center < threshold ) { // if line is below center line sensor
    sparki.moveForward();               // move forward
    speed_r = wheelSpeed;
    speed_l = wheelSpeed;
  }
  else {
    if ( line_left < threshold ) {      // if line is below left line sensor
      sparki.moveLeft(); // turn left
      speed_r = wheelSpeed;
      speed_l = -wheelSpeed;
    }
    if ( line_right < threshold ) {     // if line is below right line sensor
      sparki.moveRight(); // turn right
      speed_r = -wheelSpeed;
      speed_l = wheelSpeed;
    }
  }

  updateOdometry();
  displayOdometry();

  stopTime = millis();
  loopTime = startTime - stopTime;
  while ((stopTime - startTime) < 100) {
    stopTime = millis();
    loopTime = stopTime - startTime;
  }
}
