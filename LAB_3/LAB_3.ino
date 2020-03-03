#include <Sparki.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0275   // meters/second
#define CYCLE_TIME .050      // Default 50ms cycle time
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03    // meters

#define STOP 0
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3

#define FWD 1
#define NONE 0
#define BCK -1

// Threshold to fix bearing error
#define dist_thresh 0.03

// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

// Controller and dTheta update rule settings
int current_state = CONTROLLER_GOTO_POSITION_PART3;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0., phi_r = 0.;             // Wheel rotation (radians)

// Wheel rotation vars
float left_speed_pct = 0.;
float right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain_1 = 1.;
const float bearing_gain_1 = 1.;
const float heading_gain_1 = 0;

const float distance_gain_2 = 1.;
const float bearing_gain_2 = 0;
const float heading_gain_2 = 1.;

float dX = 0., dY = 0., dTheta = 0.;

//=====================================================================================//
// VOID SETUP
//=====================================================================================//
void setup()
{
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_dir = NONE;
  right_dir = NONE;

  // Set test cases here!
  set_pose_destination(0.15, 0.05, to_radians(15)); // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
  Serial.begin(115200);
}
//=====================================================================================//
// VOID LOOP
//=====================================================================================//

void loop()
{
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
  double time = 0.0;

  switch (current_state)
  {
    case STOP:
      sparki.moveStop();
      sparki.RGB(RGB_RED);
      break;

    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      updateOdometry();
      readSensors();
      if (line_center < threshold)
      {
        sparki.moveForward();
        left_dir = FWD;
        right_dir = FWD;
      }
      else if (line_left < threshold)
      {
        sparki.moveLeft();
        left_dir = BCK;
        right_dir = FWD;
      }
      else if (line_right < threshold)
      {
        sparki.moveRight();
        left_dir = FWD;
        right_dir = BCK;
      }
      else
      {
        sparki.moveStop();
        left_dir = NONE;
        right_dir = NONE;
      }

      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold)
      {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      }
      break;
    case CONTROLLER_GOTO_POSITION_PART2:
      updateOdometry();
      dX = dest_pose_x - pose_x;
      dY = dest_pose_y - pose_y;

      d_err = orig_dist_to_goal;            // Distance error
      b_err = heading(dX, dY) - pose_theta; // Bearing error
      h_err = dest_pose_theta - pose_theta; // Heading error

      //      Serial.print("d_err = ");
      //      Serial.print(d_err);
      //      Serial.print("  b_err = ");
      //      Serial.print(b_err);
      //      Serial.print("  h_err = ");
      //      Serial.println(h_err);

      sparki.moveRight(to_degrees(b_err)); // TURN
      delay(500);
      sparki.moveForward(d_err * 100); // MOVE
      delay(500);
      sparki.moveRight(to_degrees(d_err)); // TURN

      while (1)
      {
      }
      break;

    case CONTROLLER_GOTO_POSITION_PART3:
      updateOdometry();
      displayOdometry();
      calculateErrors();

      //      Serial.print("d_err = ");
      //      Serial.print(d_err);
      //      Serial.print("  b_err = ");
      //      Serial.print(b_err);
      //      Serial.print("  h_err = ");
      //      Serial.println(h_err);

      if (d_err < 0.02 && fabs(h_err) < to_radians(15))
      {
        current_state = STOP;
        break;
      }

      double x_dot;
      double t_dot;
      if (d_err >= 0.02)
      {
        x_dot = distance_gain_1 * d_err;                         // Forward Velocity of robot    [m]
        t_dot = bearing_gain_1 * b_err + heading_gain_1 * h_err; // Rotational velocity of robot [radians]
      }
      else
      {
        x_dot = distance_gain_2 * d_err;                         // Forward Velocity of robot    [m]
        t_dot = bearing_gain_2 * b_err + heading_gain_2 * h_err; // Rotational velocity of robot [radians]
      }

      //      Serial.print("X dot = ");
      //      Serial.print(x_dot);
      //      Serial.print(" [m/s] |  t dot = ");
      //      Serial.print(t_dot);
      //      Serial.println(" [rad/s]\n");

      phi_l = (2 * x_dot - (t_dot * AXLE_DIAMETER)) / (2 * WHEEL_RADIUS); // Calculate the left wheel speed
      phi_r = (2 * x_dot + (t_dot * AXLE_DIAMETER)) / (2 * WHEEL_RADIUS); // Calculate the right wheel speed

      // Serial.print("phi_l = ");
      // Serial.print(phi_l);
      // Serial.print(" [m/s] |  phi_r = ");
      // Serial.print(phi_r);
      // Serial.println(" [m/s]\n");

      left_speed_pct = (fabs(phi_l) >= fabs(phi_r)) ? 1 : fabs(phi_l / phi_r);
      right_speed_pct = (fabs(phi_r) >= fabs(phi_l)) ? 1 : fabs(phi_r / phi_l);

      left_dir = (phi_l >= 0) ? DIR_CCW : DIR_CW;
      right_dir = (phi_r >= 0) ? DIR_CW : DIR_CCW;

      // Serial.print("left_percentage = ");
      // Serial.print(left_speed_pct);
      // Serial.print("  | right_percentage = ");
      // Serial.println(right_speed_pct);

      begin_time = millis();
      sparki.motorRotate(MOTOR_LEFT, left_dir, abs(int(left_speed_pct * 100.)));
      sparki.motorRotate(MOTOR_RIGHT, right_dir, abs(int(right_speed_pct * 100.)));

      break;
  }

  sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();

  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000 * CYCLE_TIME)
    delay(1000 * CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}

//=====================================================================================//
// USEFUL FUNCTIONS
//=====================================================================================//
float to_radians(double deg)
{
  return deg * 3.1415 / 180.;
}

float to_degrees(double rad)
{
  return rad * 180 / 3.1415;
}

double heading(double x, double y)
{
  double head = atan2(y, x); // Slope Y, Slope X
  return head;
}

//=====================================================================================//
// SET POSE DESTINATION
//=====================================================================================//
// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t)
{
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI)
    dest_pose_theta -= 2 * M_PI;
  if (dest_pose_theta < -M_PI)
    dest_pose_theta += 2 * M_PI;
  orig_dist_to_goal = sqrt(sq(dest_pose_x - pose_x) + sq(dest_pose_y - pose_y));
}

void readSensors()
{
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void calculateErrors()
{
  dX = dest_pose_x - pose_x; // Calculate the X displacement
  dY = dest_pose_y - pose_y; // Calculate the Y displacement

  d_err = sqrt(sq(dX) + sq(dY));        // Distance error [p]     [m]
  b_err = heading(dX, dY) - pose_theta; // Bearing error  [alpha] [rad]
  h_err = dest_pose_theta - pose_theta; // Heading error  [n]     [rad]

  if (b_err > M_PI)
    b_err -= 2. * M_PI;
  if (b_err <= -M_PI)
    b_err += 2. * M_PI;

  if (h_err > M_PI)
    h_err -= 2. * M_PI;
  if (h_err <= -M_PI)
    h_err += 2. * M_PI;
}

void updateOdometry()
{
  double speed_r = (right_dir) * right_speed_pct * ROBOT_SPEED;
  double speed_l = (!left_dir) * left_speed_pct * ROBOT_SPEED;

  double x_dot = ((speed_r + speed_l) / 2) * cos(pose_theta);
  double y_dot = ((speed_r + speed_l) / 2) * sin(pose_theta);
  double theta_dot = ((speed_r - speed_l) / (AXLE_DIAMETER));

  pose_x = pose_x + x_dot * CYCLE_TIME;
  pose_y = pose_y + y_dot * CYCLE_TIME;
  pose_theta = pose_theta + theta_dot * CYCLE_TIME;

  // Bound theta
  if (pose_theta > M_PI)
    pose_theta -= 2. * M_PI;
  if (pose_theta <= -M_PI)
    pose_theta += 2. * M_PI;
}

void displayOdometry()
{
  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y);
  sparki.print("T: ");
  sparki.print(to_degrees(pose_theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(dest_pose_theta));

  sparki.print("dX : ");
  sparki.print(dX);
  sparki.print("   dT: ");
  sparki.println(dTheta);
  sparki.print("phl: ");
  sparki.print(phi_l);
  sparki.print(" phr: ");
  sparki.println(phi_r);
  sparki.print("p: ");
  sparki.print(d_err);
  sparki.print(" a: ");
  sparki.println(to_degrees(b_err));
  sparki.print("h: ");
  sparki.println(to_degrees(h_err));
}
