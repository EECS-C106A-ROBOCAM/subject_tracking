#include <Servo.h>

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;
const char ROS_TOPIC_NAME[] = "/arduino_joints";

const short PIN_BASE_YAW = 8;
const short PIN_BOT_4B = 9;
const short PIN_TOP_4B = 10;
const short PIN_CAM_PITCH = 11;
const short PIN_CAM_YAW = 12;
const short PIN_CAM_ROLL = 7;

Servo baseYawServo;
Servo bot4BServo;
Servo top4BServo;
Servo camPitchServo;
Servo camYawServo;
Servo camRollServo;

// Base Yaw Servo
const short BASE_YAW_MIN = 40;
const short BASE_YAW_MAX = 120;

// Bottom 4 Bar Servo
const short BOT_4B_MIN = 70;
const short BOT_4B_MAX = 130;

// Top 4 Bar Servo
const short TOP_4B_MIN = 25;
const short TOP_4B_MAX = 105;

// Camera Pitch Servo
const short CAM_PITCH_MIN = 0;
const short CAM_PITCH_MAX = 120;

// Camera Yaw Servo
const short CAM_YAW_MIN = 0;
const short CAM_YAW_MAX = 180;

// Camera Roll Servo
const short CAM_ROLL_MIN = 0;
const short CAM_ROLL_MAX = 180;

const short MIN_ANGLES[6] = { BASE_YAW_MIN, BOT_4B_MIN, TOP_4B_MIN, CAM_PITCH_MIN, CAM_YAW_MIN, CAM_ROLL_MIN};
const short MAX_ANGLES[6] = { BASE_YAW_MAX, BOT_4B_MAX, TOP_4B_MAX, CAM_PITCH_MAX, CAM_YAW_MAX, CAM_ROLL_MAX};

Servo* SERVOS[6] = {&baseYawServo, &bot4BServo, &top4BServo, &camPitchServo, &camYawServo, &camRollServo };

bool inBounds(short* targetAngles) {
  bool success = true;
  for (int i = 0; i < 6; ++i) {
    success &= (targetAngles[i] >= MIN_ANGLES[i]) && (targetAngles[i] <= MAX_ANGLES[i]);
  }
  return success;
}

/**
   Converts the 6 joint inputs into 6 motor angles.
   @param jointInputArray Length 6 array of joint input angles
   @param motorOutputArray Length 6 array of motor output angles
   @return true if motor angles are in range and valid; false otherwise
*/
bool jointToMotorAngles(float* jointInputArray, short* motorOutputArray) {
  String motorString = "Motor Angles: ";
  
  for (int i = 0; i < 6; ++i) {
    float angle = jointInputArray[i] * RAD_TO_DEG; // Convert radians to degrees

    if (i >= 0 && i < 3) {
      // For DS3218 servos, rescale out of 270
      angle *= 180.0 / 270.0;
    }

    if (i >= 1 && i < 4) {
      // For pitch and four-bar joints, invert angle
      angle *= -1;
    }

    angle += 90; // Since 90 is the center of the range

    motorOutputArray[i] = short(angle); // Narrowing from float to short
    
    motorString = motorString + motorOutputArray[i] + " ";
  }
  nh.loginfo(motorString.c_str());

  return inBounds(motorOutputArray);
}

void glide(short* targetAngles, int duration, int step = 10) {
  float currentAngles[6];
  float stepSizes[6];

  if (!inBounds(targetAngles)) {
    nh.logerror("Motors out of range!");
    return;
  }

  int numSteps = duration / step;
  for (int i = 0; i < 6; ++i) {
    currentAngles[i] = SERVOS[i]->read();
    stepSizes[i] = (targetAngles[i] - currentAngles[i]) / numSteps;
  }

  for (int s = 0; s < numSteps; ++s) {
    for (int i = 0; i < 6; ++i) {
      currentAngles[i] += stepSizes[i];
      SERVOS[i]->write(round(currentAngles[i]));
    }

    delay(step);
  }

  for (int i = 0; i < 6; ++i) {
    SERVOS[i]->write(targetAngles[i]);
  }
}

void motorCallback(const std_msgs::Float32MultiArray& floatArrayMsg) {
  nh.loginfo("Running callback now!");

  float* jointAngles = floatArrayMsg.data;
  short motorAngles[6];
  if (!jointToMotorAngles(jointAngles, motorAngles)) {
    // Error: the joint angles we received force the motor out of range!
    nh.logerror("Motors out of range!");
    return;
  }
  nh.loginfo("Moving motors now!");
  glide(motorAngles, 100);
}

ros::Subscriber<std_msgs::Float32MultiArray> sub(ROS_TOPIC_NAME, motorCallback);

void setup() {
  // ROS initialization
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.loginfo("Connected!");

  // Attach servo objects to pins
  baseYawServo.attach(PIN_BASE_YAW);
  bot4BServo.attach(PIN_BOT_4B);
  top4BServo.attach(PIN_TOP_4B);
  camPitchServo.attach(PIN_CAM_PITCH);
  camYawServo.attach(PIN_CAM_YAW);
  camRollServo.attach(PIN_CAM_ROLL);

  baseYawServo.write(90);
  bot4BServo.write(90);
  top4BServo.write(90);
  camPitchServo.write(90);
  camYawServo.write(90);
  camRollServo.write(90);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
