#include <Ps3Controller.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// Default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Kinematics definitions
#define left_front 0
#define left_back 1
#define right_front 2
#define right_back 3

int right_legs[2] = {right_front, right_back};

// Leg dimensions
#define link_1 5
#define link_2 8
#define link_3 12

float phi = M_PI/2;

// Body dimensions
#define length 1
#define width 1
#define height 1

// Leg origins (left_f, left_b, right_b, right_f), i.e., the coordinate of j1
double leg_origins[4][3] = {
    {length / 2, width / 2, 0},
    {-length / 2, width / 2, 0},
    {-length / 2, -width / 2, 0},
    {length / 2, -width / 2, 0}
};

struct reshapeArray 
{
    float result[3][1];
};

struct Array3
{
  float result[3];
};

struct Array3x3
{
  float result[3][3];
};

struct Array3 leg_IK(float xyz[3], float rot[3], int legID, bool is_radians, float center_offset[3]) {
    
    bool is_right = (legID == right_legs[0] || legID == right_legs[1]);

    float XYZ[3];

    // Add offset of each leg from the axis of rotation
    float rot_mtx[3][3];

    Array3x3 rot_result = RotMatrix3D(rot, is_radians);
    Array3x3 inv_rot_result = invertMatrix(rot_result.result);
    Array3 XYZ_result = multiplyMatrixVector(inv_rot_result.result, xyz);
    
    float inv_rot_mtx[3][3];

    // Copy values manually
    for (int i = 0; i < 3; i++) {
        XYZ[i] = XYZ_result.result[i];  // Copy XYZ result

        for (int j = 0; j < 3; j++) {
            rot_mtx[i][j] = rot_result.result[i][j];  // Copy rotation matrix
            inv_rot_mtx[i][j] = inv_rot_result.result[i][j];  // Copy inverted matrix
        }
    }

    for (int i = 0; i < 3; i++) {
        XYZ[i] += leg_origins[legID][i] - center_offset[i];
    }

    float xyz_[3];
    for (int i = 0; i < 3; i++) {
        xyz_[i] = XYZ[i] - leg_origins[legID][i] + center_offset[i];
    }

    // Calculate the angles and coordinates of the leg relative to the origin of the leg
    return leg_IK_calc(xyz_, is_right);
}

struct Array3 leg_IK_calc(float xyz[3], bool is_right) {
    float x = xyz[0], y = xyz[1], z = xyz[2];

    // Length of vector projected on the YZ plane
    float len_A = sqrt((y * y) + (z * z));

    // Angles
    float a_2 = asin((sin(phi)) * (link_1 / len_A));
    float a_3 = M_PI - phi - a_2;

    float theta_1;
    float a_1 = point_to_rad(y,z);

    theta_1 = a_1 - ((2 * M_PI) - a_3);

    float j2[3] = {0, link_1 * cos(theta_1), link_1 * sin(theta_1)};
    float j4[3] = {x, y, z};
    float j4_2_vec[3] = {j4[0] - j2[0], j4[1] - j2[1], j4[2] - j2[2]};

    float rot[] = {-theta_1, 0, 0};
    float rot_mtx[3][3];
    struct Array3x3 rot_result = RotMatrix3D(rot, true);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rot_mtx[i][j] = rot_result.result[i][j];  // Copy rotation matrix
        }
    }

    struct reshapeArray j4_2_vec_reshaped = reshape(j4_2_vec, 3, 1);

    float j4_2_vec_final[3];
    for (int i = 0; i < 3; i++) {
        j4_2_vec_final[i] = 0;  // Initialize to zero
        for (int j = 0; j < 3; j++) {
          j4_2_vec_final[i] += rot_mtx[i][j] * j4_2_vec_reshaped.result[j][0]; 
        }
    }

    
    // xyz in the rotated coordinate system + offset due to link_1 removed
    float x_ = j4_2_vec_final[0];
    float y_ = j4_2_vec_final[1];
    float z_ = j4_2_vec_final[2];
    
    float len_B = sqrt((x_* x_) + (z_ * z_)); // norm(j4-j2)
    
    // handling mathematically invalid input, i.e., point too far away to reach
    if (len_B >= (link_2 + link_3)) {
        len_B = (link_2 + link_3) * 0.99999;
        Serial.println("target coordinate too far away");
    }
    
    // b_1 : angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
    // b_2 : angle between len_B and link_2
    // b_3 : angle between link_2 and link_3

    float b_1 = point_to_rad(x_,z_);
    float b_2 = acos(((link_2*link_2) + (len_B*len_B) - (link_3*link_3)) / (2 * link_2 * len_B));
    float b_3 = acos(((link_2*link_2) + (link_3*link_3) - (len_B*len_B)) / (2 * link_2 * link_3));  

    // assuming theta_2 = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
    float theta_2 = M_PI - (b_1 - b_2);
    float theta_3 = M_PI - b_3;
    
    // modify angles to match robot's configuration (i.e., adding offsets)

    return angle_corrector(theta_1, theta_2, theta_3, is_right);
}

struct reshapeArray reshape(float input[3], int rows, int cols) {
    struct reshapeArray results;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            results.result[i][j] = input[i * cols + j];
        }
    }
    return results;
}
struct Array3 multiplyMatrixVector(float m[3][3], float vec[3]) {
    struct Array3 results;
    for (int i = 0; i < 3; i++) {
        results.result[i] = 0;
        for (int j = 0; j < 3; j++) {
            results.result[i] += m[i][j] * vec[j];
        }
    }
    return results;
}

struct Array3 angle_corrector(float a, float b, float c, bool is_right) {
    struct Array3 results;
    
    float theta_1;
    float theta_2; 
    float theta_3;

    if (is_right){
      theta_1 = a + M_PI/2;
      theta_2 = b + M_PI/2;
      theta_3 = c;
    } else {
      theta_1 = (a + M_PI/2);
      theta_2 = -(b + M_PI/2);
      theta_3 = -c;
    }

    results.result[0] = theta_1;
    results.result[1] = theta_2;
    results.result[2] = theta_3;

    return results;
}

struct Array3x3 RotMatrix3D(float rotation[3], bool is_radians) {
    float roll = rotation[0];
    float pitch = rotation[1];
    float yaw = rotation[2];

    // convert to radians if the input is in degrees
    if (!is_radians) {
        roll = radians(roll);
        pitch = radians(pitch);
        yaw = radians(yaw);
    }

    // rotation matrix about each axis
    float rotX[3][3] = {
        {1, 0, 0},
        {0, cos(roll), -sin(roll)},
        {0, sin(roll), cos(roll)}
    };

    float rotY[3][3] = {
        {cos(pitch), 0, sin(pitch)},
        {0, 1, 0},
        {-sin(pitch), 0, cos(pitch)}
    };

    float rotZ[3][3] = {
        {cos(yaw), -sin(yaw), 0},
        {sin(yaw), cos(yaw), 0},
        {0, 0, 1}
    };

    struct Array3x3 rotationMatrix;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rotationMatrix.result[i][j] = (i == j) ? 1 : 0;
        }
    }
  
    rotationMatrix = multiplyMatrices(rotZ, rotY);
    return multiplyMatrices(rotationMatrix.result, rotX);
}

// Matrix multiplication (3x3)
struct Array3x3 multiplyMatrices(float a[3][3], float b[3][3]) {
    struct Array3x3 results;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            results.result[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                results.result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return results;
}

// Matrix inversion (assuming orthogonal)
struct Array3x3 invertMatrix(float matrix[3][3]) {
    struct Array3x3 results;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            results.result[i][j] = matrix[j][i];
        }
    }
  return results;
}

float point_to_rad(float p1, float p2) { // converts 2D cartesian points to polar angles in range 0 - 2pi
    if (p1 > 0 && p2 >= 0) {
        return atan(p2 / p1);
    } else if (p1 == 0 && p2 > 0) {
        return (PI / 2);
    } else if (p1 < 0 && p2 >= 0) {
        return (M_PI - (atan(abs(p2 / p1))));
    } else if (p1 < 0 && p2 < 0) {
        return M_PI + (atan(p2 / p1));
    } else if (p1 > 0 && p2 < 0) {
        return ((2 * M_PI) - (atan(abs(p2 / p1))));
    } else if (p1 == 0 && p2 < 0) {
        return ((PI * 3) / 2);
    } else if (p1 == 0 && p2 == 0) {
        return 0; // or return PI * 3 / 2 if you prefer
    }
    return 0; // default return value
}

// PWM and servo control definitions
#define SERVOMIN  90  // Minimum pulse length count (out of 4096)
#define SERVOMAX  553  // Maximum pulse length count (out of 4096)
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

uint16_t servonum = 0;

// PS3 Controller related

// -8,5,-13.5 neutral

int battery = 0;
float x = -5;
float y = 5; 
float z = -13;

int frame = 0;

int getPulse(int angle) 
{
  if (angle < 0) {
    angle = 180 + angle; // Convert negative angles to positive equivalent
  }

  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}


void setup() {
    Serial.begin(115200);

    // Controller setup
    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin("00:00:00:00:00:00");

    Serial.println("Ready.");
    Ps3.setPlayer(1);

    // Motor setup
    pwm.begin();
    pwm.setOscillatorFrequency(27000000); // 23-27
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

    for (int i = 0; i < 12; i++){
        pwm.setPWM(i, 0, (SERVOMIN + SERVOMAX) / 2);
    }

    delay(10);
}

float angles1[3] = {0,0,0};
float angles2[3] = {0,0,0};
float angles3[3] = {0,0,0};
float angles4[3] = {0,0,0};

void loop() {

    float Ps3X = Ps3.data.analog.stick.ry/20.0;
    float Ps3Y = Ps3.data.analog.stick.rx/20.0;

    if (Ps3X < 0.5 && Ps3X > -0.5){
      Ps3X = 0;
    }

    if (Ps3Y < 0.5 && Ps3Y > -0.5){
      Ps3Y = 0;
    }

    if (Ps3Y == 0 && Ps3X == 0){
      frame = 0;
    }

    float x1;
    float x2;
    float y1;
    float y2;
    float z1; 
    float z2;

    float xorig = x - Ps3X;
    float xorigNeg = x + Ps3X;

    float yorig = y - Ps3Y;
    float yorigNeg = y + Ps3Y;

    switch (frame) {
      case (0):
        x1 = x;
        x2 = x;
        y1 = y;
        y2 = y;
        z1 = z;
        z2 = z;
        break;
      case (1):
        x1 = xorig/3;
        x2 = xorigNeg/3;
        y1 = yorig/3;
        y2 = yorigNeg/3;
        z1 = z + 1.5;
        z2 = z - 1;
        break;
      case (2):
        x1 = xorig/2;
        x2 = xorigNeg/2;
        y1 = yorig/2;
        y2 = yorigNeg/2;
        z1 = z + 4;
        z2 = z - 4;
        break;
      case (3):
        x1 = (xorig*2)/3;
        x2 = (xorigNeg*2)/3;
        y1 = (yorig*2)/3;
        y2 = (yorigNeg*2)/3;
        z1 = z + 2;
        z2 = z - 2;
        break;
      case (4):
        x1 = xorigNeg;
        x2 = xorig;
        y1 = yorigNeg;
        y2 = yorig;
        z1 = z;
        z2 = z;
        break;
      case (5):
        x1 = (xorigNeg*2)/3;
        x2 = (xorig*2)/3;
        y1 = (yorigNeg*2)/3;
        y2 = (yorig*2)/3;
        z1 = z + 2;
        z2 = z - 2;
        break;
      case (6):
        x1 = xorigNeg/2;
        x2 = xorig/2;
        y1 = yorigNeg/2;
        y2 = yorig/2;
        z1 = z + 4;
        z2 = z - 4;
        break;
      case (7):
        x1 = xorigNeg/3;
        x2 = xorig/3;
        y1 = yorigNeg/3;
        y2 = yorig/3;
        z1 = z + 2;
        z2 = z - 2;
        break;
      case (8):
        x1 = x;
        x2 = x;
        y1 = y;
        y2 = y;
        z1 = z;
        z2 = z;
        break;
    }

    float target[3] = {x1, y1, z1};  // Target foot position (X, Y, Z)
    float targetNegative[3] = {x2, y2, z2};

    struct Array3 FRLeg;
    struct Array3 BLLeg;

    struct Array3 FLLeg;
    struct Array3 BRLeg;

    if (frame != 0){
      if (frame < 4){
        FRLeg = leg_IK_calc(targetNegative, true);
        BLLeg = leg_IK_calc(targetNegative, false);

        FLLeg = leg_IK_calc(target, false);
        BRLeg = leg_IK_calc(target, true);
      } else {
        FRLeg = leg_IK_calc(target, true);
        BLLeg = leg_IK_calc(target, false);

        FLLeg = leg_IK_calc(targetNegative, false);
        BRLeg = leg_IK_calc(targetNegative, true);
      }
    } else {
      FRLeg = leg_IK_calc(target, true);
      BLLeg = leg_IK_calc(target, false);

      FLLeg = leg_IK_calc(target, false);
      BRLeg = leg_IK_calc(target, true);
    }

     for (int i = 0; i < 3; i++) {
         angles1[i] = FRLeg.result[i];
         angles2[i] = FLLeg.result[i];
         angles3[i] = BRLeg.result[i];
         angles4[i] = BLLeg.result[i];
     };

     // convert degrees

     for (int i = 0; i < 3; i++){
       angles1[i] *= (180/M_PI);
       angles2[i] *= (180/M_PI);
       angles3[i] *= (180/M_PI);
       angles4[i] *= (180/M_PI);
     };

     //Serial.print("Target 1: "); Serial.print(target[0]); Serial.print("Target 2: "); Serial.print(target[1]); Serial.print("Target 3: "); Serial.println(target[2]);

     Serial.print("a 1: "); Serial.print(angles1[0]); Serial.print("a 2: "); Serial.print(angles1[1]); Serial.print("a 3: "); Serial.println(angles1[2]);
     Serial.print("b 1: "); Serial.print(angles2[0]); Serial.print("b 2: "); Serial.print(angles2[1]); Serial.print("b 3: "); Serial.println(angles2[2]);



    pwm.setPWM(0, 0, getPulse(angles1[0] - 4));
    pwm.setPWM(1, 0, getPulse(angles1[1] - 8));
    pwm.setPWM(2, 0, getPulse(angles1[2] - 7));

    pwm.setPWM(3, 0, getPulse(angles2[0] - 2));
    pwm.setPWM(4, 0, getPulse(angles2[1] - 11));
    pwm.setPWM(5, 0, getPulse(angles2[2] - 14));

    pwm.setPWM(6, 0, getPulse(angles3[0] - 8));
    pwm.setPWM(7, 0, getPulse(angles3[1] - 5));
    pwm.setPWM(8, 0, getPulse(angles3[2] - 5));

    pwm.setPWM(9, 0, getPulse(angles4[0] - 6));
    pwm.setPWM(10, 0, getPulse(angles4[1] - 1));
    pwm.setPWM(11, 0, getPulse(angles4[2] - 10));
    

    if (frame < 8){
      frame++;
    } else {
      frame = 0;
    }
    delay(100);
}

void notify() {
    if (battery != Ps3.data.status.battery) {
        battery = Ps3.data.status.battery;
        Serial.print("The controller battery is ");
        if (battery == ps3_status_battery_charging) Serial.println("charging");
        else if (battery == ps3_status_battery_full) Serial.println("FULL");
        else if (battery == ps3_status_battery_high) Serial.println("HIGH");
        else if (battery == ps3_status_battery_low) Serial.println("LOW");
        else if (battery == ps3_status_battery_dying) Serial.println("DYING");
        else if (battery == ps3_status_battery_shutdown) Serial.println("SHUTDOWN");
        else Serial.println("UNDEFINED");
    }
}

void onConnect() {
    Serial.println("Connected.");
}
