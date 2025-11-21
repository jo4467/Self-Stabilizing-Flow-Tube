#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <utility/imumaths.h>

// sensor setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Servo object creation
Servo servo1; //left servo controls pitch
Servo servo2; //right servo controls pitch
Servo servo3; //top servo controls yaw
Servo servo4; //bottom servo controls yaw
int edfpin = 10;
Servo edf;

// desired orientation
double pitch_setpoint = 0;
double yaw_setpoint = 0;

// PID Constants for Pitch
double Kp_pitch = 1.0;
double Ki_pitch = 0.1;
double Kd_pitch = 0.5;

// PID pitch variables
double error_pitch, last_error_pitch, integral_pitch, derivative_pitch,
    pitch_output;

// PID Constants for Yaw
double Kp_yaw = 1.0;
double Ki_yaw = 0.1;
double Kd_yaw = 0.5;

// PID yaw variables
double error_yaw, last_error_yaw, integral_yaw, derivative_yaw, yaw_output;

// current system output angle
float current_pitch = 0;
float current_effective_yaw = 0;

// function declaration
void setupServos();
bool readSensor();
void computePID(double setpoint, double input, double Kp, double Ki, double Kd,
                double &output, double &error, double &last_error,
                double &integral, double &derivative);
void setServoPositions(double pitch_pid_output, double yaw_pid_output);

//pre main code setup
void setup() {
  Serial.begin(115200);
  Serial.println("Dual-Axis PID Servo Controller Initializing...");

  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  setupServos();
  edf.attach(edfpin);
  edf.writeMicroseconds(1000);
  delay(3000);
  edf.writeMicroseconds(1200);
}

void loop() {
  if (readSensor()) {
    // Compute PID for Pitch
    computePID(pitch_setpoint, current_pitch, Kp_pitch, Ki_pitch, Kd_pitch,
               pitch_output, error_pitch, last_error_pitch, integral_pitch,
               derivative_pitch);

    // Compute PID for Yaw
    computePID(yaw_setpoint, current_effective_yaw, Kp_yaw, Ki_yaw, Kd_yaw, yaw_output,
               error_yaw, last_error_yaw, integral_yaw, derivative_yaw);

    // Update servo positions based on PID outputs
    setServoPositions(pitch_output, yaw_output);

    // debugging
    Serial.print("Pitch: ");
    Serial.print(current_pitch);
    Serial.print(" | Pitch_PID: ");
    Serial.print(pitch_output);
    Serial.print(" | Yaw: ");
    Serial.print(current_effective_yaw);
    Serial.print(" | Yaw_PID: ");
    Serial.println(yaw_output);
  }

  delay(10); // Small delay for stability
}

//helper code

//initializes servos
void setupServos() {
  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
}

//code to read the sensor
bool readSensor() {
  sensors_event_t event;
  bno.getEvent(&event);
  current_pitch = event.orientation.z - 1.94;
  current_effective_yaw = event.orientation.x;
  if (current_effective_yaw > 180) {
    current_effective_yaw = current_effective_yaw - 360.0;
  }
  return true;
}

//code to compute PID
void computePID(double setpoint, double input, double Kp, double Ki, double Kd,
                double &output, double &error, double &last_error,
                double &integral, double &derivative) {
  error = setpoint - input;
  integral += error;
  derivative = error - last_error;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  last_error = error;

  // integral overshoot protection
  if (integral > 100)
    integral = 100;
  if (integral < -100)
    integral = -100;
}


//code to set the servo positions
void setServoPositions(double pitch_pid_output, double yaw_pid_output) {
  int servo1_pos, servo2_pos, servo3_pos, servo4_pos;
  int neutral_pos = 90;

  servo1_pos = neutral_pos + pitch_pid_output; //left servo
  servo2_pos = neutral_pos - pitch_pid_output; // right servo

  servo3_pos = neutral_pos + yaw_pid_output; //top servo
  servo4_pos = neutral_pos - yaw_pid_output; // bottom servo

  // Constrain the servo positions to the valid range [0, 180] (2 seems to work better)
  servo1_pos = constrain(servo1_pos, 2, 180);
  servo2_pos = constrain(servo2_pos, 2, 180);
  servo3_pos = constrain(servo3_pos, 2, 180);
  servo4_pos = constrain(servo4_pos, 2, 180);

  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
  servo3.write(servo3_pos);
  servo4.write(servo4_pos);
  Serial.print("servo1: ");
  Serial.print(servo1_pos);
  Serial.print("servo2: ");
  Serial.print(servo2_pos);
  Serial.print("servo3: ");
  Serial.print(servo3_pos);
  Serial.print("servo4: ");
  Serial.println(servo4_pos);
}
