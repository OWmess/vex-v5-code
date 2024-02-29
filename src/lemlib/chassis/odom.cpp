
// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include <math.h>
#include "pros/rtos.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/llemu.hpp"
// tracking thread
pros::Task* trackingTask = nullptr;

// global variables
lemlib::OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr, nullptr); // the sensors to be used for odometry
lemlib::Drivetrain drive(nullptr, nullptr, 0, 0, 0, 0); // the drivetrain to be used for odometry
lemlib::Pose odomPose(0, 0, 0); // the pose of the robot
lemlib::Pose odomSpeed(0, 0, 0); // the speed of the robot
lemlib::Pose odomLocalSpeed(0, 0, 0); // the local speed of the robot




float prevVertical = 0;
float prevVertical1 = 0;
float prevVertical2 = 0;
float prevHorizontal = 0;
float prevHorizontal1 = 0;
float prevHorizontal2 = 0;
float prevImu = 0;


/**
 * @brief convert meter to inch
 * @param meter
 * @return inch
 */
template<typename T>
inline T meter2Inch(const T &meter) {
  return meter / 0.0254;
}


double calculateTotalRotation(){
    // 计算自开机后的旋转度数，考虑循环情况
    double currentAngle = lemlib::gps->get_heading();
    static double totalRotation = 0;
    static double prevAngle = 0;
    if(prevAngle==0){
        while(errno==EAGAIN){
            pros::delay(10);
        }
        totalRotation=lemlib::gps->get_heading()-180;
    }
    if(prevAngle!=0&&errno!=EAGAIN){
        totalRotation+=currentAngle-prevAngle;
        if(prevAngle>300&&currentAngle<60){
            totalRotation+=360;
            printf("totalRotation+=360\n total:%lf\n",totalRotation);
        }
        if(prevAngle<60&&currentAngle>300){
            totalRotation-=360;
            printf("totalRotation-=360\n total:%lf\n",totalRotation);

        }
    }
    prevAngle=currentAngle;
    return totalRotation;
}


/**
 * @brief Set the sensors to be used for odometry
 *
 * @param sensors the sensors to be used
 * @param drivetrain drivetrain to be used
 */
void lemlib::setSensors(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain) {
    odomSensors = sensors;
    drive = drivetrain;
}

/**
 * @brief Get the pose of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return Pose
 */
lemlib::Pose lemlib::getPose(bool radians) {
    if (radians) return odomPose;
    else return lemlib::Pose(odomPose.x, odomPose.y, radToDeg(odomPose.theta));
}

/**
 * @brief Set the Pose of the robot
 *
 * @param pose the new pose
 * @param radians true if theta is in radians, false if in degrees. False by default
 */
void lemlib::setPose(lemlib::Pose pose, bool radians) {
    if (radians) odomPose = pose;
    else odomPose = lemlib::Pose(pose.x, pose.y, degToRad(pose.theta));
}

/**
 * @brief Get the speed of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return lemlib::Pose
 */
lemlib::Pose lemlib::getSpeed(bool radians) {
    if (radians) return odomSpeed;
    else return lemlib::Pose(odomSpeed.x, odomSpeed.y, radToDeg(odomSpeed.theta));
}

/**
 * @brief Get the local speed of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return lemlib::Pose
 */
lemlib::Pose lemlib::getLocalSpeed(bool radians) {
    if (radians) return odomLocalSpeed;
    else return lemlib::Pose(odomLocalSpeed.x, odomLocalSpeed.y, radToDeg(odomLocalSpeed.theta));
}

/**
 * @brief Estimate the pose of the robot after a certain amount of time
 *
 * @param time time in seconds
 * @param radians False for degrees, true for radians. False by default
 * @return lemlib::Pose
 */
lemlib::Pose lemlib::estimatePose(float time, bool radians) {
    // get current position and speed
    Pose curPose = getPose(true);
    Pose localSpeed = getLocalSpeed(true);
    // calculate the change in local position
    Pose deltaLocalPose = localSpeed * time;

    // calculate the future pose
    float avgHeading = curPose.theta + deltaLocalPose.theta / 2;
    Pose futurePose = curPose;
    futurePose.x += deltaLocalPose.y * sin(avgHeading);
    futurePose.y += deltaLocalPose.y * cos(avgHeading);
    futurePose.x += deltaLocalPose.x * -cos(avgHeading);
    futurePose.y += deltaLocalPose.x * sin(avgHeading);
    if (!radians) futurePose.theta = radToDeg(futurePose.theta);

    return futurePose;
}

/**
 * @brief Update the pose of the robot
 *
 */
void lemlib::update() {
    // TODO: add particle filter
    // get the current sensor values
    float vertical1Raw = 0;
    float vertical2Raw = 0;
    float horizontal1Raw = 0;
    float horizontal2Raw = 0;
    float imuRaw = 0;
    if (odomSensors.vertical1 != nullptr) vertical1Raw = odomSensors.vertical1->getDistanceTraveled();
    if (odomSensors.vertical2 != nullptr) vertical2Raw = odomSensors.vertical2->getDistanceTraveled();
    if (odomSensors.horizontal1 != nullptr) horizontal1Raw = odomSensors.horizontal1->getDistanceTraveled();
    if (odomSensors.horizontal2 != nullptr) horizontal2Raw = odomSensors.horizontal2->getDistanceTraveled();
    ///TODO: 此处暂时使用gps的yaw角度代替imu的yaw角度
    if (odomSensors.imu != nullptr){ 
        imuRaw = degToRad(calculateTotalRotation());
    }

    // calculate the change in sensor values
    float deltaVertical1 = vertical1Raw - prevVertical1;
    float deltaVertical2 = vertical2Raw - prevVertical2;
    float deltaHorizontal1 = horizontal1Raw - prevHorizontal1;
    float deltaHorizontal2 = horizontal2Raw - prevHorizontal2;

    float deltaImu = imuRaw - prevImu;

    // update the previous sensor values
    prevVertical1 = vertical1Raw;
    prevVertical2 = vertical2Raw;
    prevHorizontal1 = horizontal1Raw;
    prevHorizontal2 = horizontal2Raw;
    prevImu = imuRaw;

    // calculate the heading of the robot
    // Priority:
    // 1. Horizontal tracking wheels
    // 2. Vertical tracking wheels
    // 3. Inertial Sensor
    // 4. Drivetrain
    float heading = odomPose.theta;
    // calculate the heading using the horizontal tracking wheels
    if (odomSensors.horizontal1 != nullptr && odomSensors.horizontal2 != nullptr)
        heading -= (deltaHorizontal1 - deltaHorizontal2) /
                   (odomSensors.horizontal1->getOffset() - odomSensors.horizontal2->getOffset());
    // else, if both vertical tracking wheels aren't substituted by the drivetrain, use the vertical tracking wheels
    else if (!odomSensors.vertical1->getType() && !odomSensors.vertical2->getType())
        heading -= (deltaVertical1 - deltaVertical2) /
                   (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());
    // else, if the inertial sensor exists, use it
    else if (odomSensors.imu != nullptr) heading += deltaImu;
    // else, use the the substituted tracking wheels
    else
        heading -= (deltaVertical1 - deltaVertical2) /
                   (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());

    float deltaHeading = heading - odomPose.theta;
    float avgHeading = odomPose.theta + deltaHeading / 2;



    // choose tracking wheels to use
    // Prioritize non-powered tracking wheels
    lemlib::TrackingWheel* verticalWheel = nullptr;
    lemlib::TrackingWheel* horizontalWheel = nullptr;
    if (!odomSensors.vertical1->getType()) verticalWheel = odomSensors.vertical1;
    else if (!odomSensors.vertical2->getType()) verticalWheel = odomSensors.vertical2;
    else verticalWheel = odomSensors.vertical1;
    if (odomSensors.horizontal1 != nullptr) horizontalWheel = odomSensors.horizontal1;
    else if (odomSensors.horizontal2 != nullptr) horizontalWheel = odomSensors.horizontal2;
    float rawVertical = 0;
    float rawHorizontal = 0;
    if (verticalWheel != nullptr) rawVertical = verticalWheel->getDistanceTraveled();
    if (horizontalWheel != nullptr) rawHorizontal = horizontalWheel->getDistanceTraveled();
    float horizontalOffset = 0;
    float verticalOffset = 0;
    if (verticalWheel != nullptr) verticalOffset = verticalWheel->getOffset();
    if (horizontalWheel != nullptr) horizontalOffset = horizontalWheel->getOffset();

    // calculate change in x and y
    float deltaX = 0;
    float deltaY = 0;
    if (verticalWheel != nullptr) deltaY = rawVertical - prevVertical;
    if (horizontalWheel != nullptr) deltaX = rawHorizontal - prevHorizontal;
    prevVertical = rawVertical;
    prevHorizontal = rawHorizontal;

    // calculate local x and y
    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0) { // prevent divide by 0
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
    }

    // save previous pose
    lemlib::Pose prevPose = odomPose;

    // calculate global x and y
    odomPose.x += localY * sin(avgHeading);
    odomPose.y += localY * cos(avgHeading);
    odomPose.x += localX * -cos(avgHeading);
    odomPose.y += localX * sin(avgHeading);
    odomPose.theta = heading;

    // calculate speed
    odomSpeed.x = ema((odomPose.x - prevPose.x) / 0.01, odomSpeed.x, 0.95);
    odomSpeed.y = ema((odomPose.y - prevPose.y) / 0.01, odomSpeed.y, 0.95);
    odomSpeed.theta = ema((odomPose.theta - prevPose.theta) / 0.01, odomSpeed.theta, 0.95);

    // calculate local speed
    odomLocalSpeed.x = ema(localX / 0.01, odomLocalSpeed.x, 0.95);
    odomLocalSpeed.y = ema(localY / 0.01, odomLocalSpeed.y, 0.95);
    odomLocalSpeed.theta = ema(deltaHeading / 0.01, odomLocalSpeed.theta, 0.95);

    Eigen::VectorXd estimate(4);
    Eigen::MatrixXd r(4,4);
    auto [x,y,pitch,roll,yaw]=gps->get_status();
    double gpsRMSError=gps->get_error();
    estimate<< meter2Inch(x), meter2Inch(y),odomSpeed.x,odomSpeed.y;
    r<<gpsRMSError,0,0,0,
       0,gpsRMSError,0,0,
       0,0,0.001,0,
       0,0,0,0.001;
    kalmanFilter.update(estimate,r);
}

/**
 * @brief Initialize the odometry system
 *
 */
void lemlib::init() {
    if (trackingTask == nullptr) {        
        trackingTask = new pros::Task {[=] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
}
void lemlib::kalmanFilterInit() {
  /**
   * 状态向量 X(1,6)：[x, x', x'', y, y', y'']
   */
  /**
   *  F:状态转移矩阵
   */
  F << 1, dt, 0.5 * pow(dt, 2), 0, 0, 0,
      0, 1, dt, 0, 0, 0,
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, dt, 0.5 * pow(dt, 2),
      0, 0, 0, 0, 1, dt,
      0, 0, 0, 0, 0, 1;
  /**
   * H:观测矩阵
   */
  H << 1, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0;

  // 估计过程噪声协方差
  Q << pow(dt, 4) / 4.f, pow(dt, 3) / 2.f, pow(dt, 2) / 2.f, 0, 0, 0,
      pow(dt, 3) / 2.f, pow(dt, 2), dt, 0, 0, 0,
      pow(dt, 2) / 2.f, dt, 1, 0, 0, 0,
      0, 0, 0, pow(dt, 4) / 4.f, pow(dt, 3) / 2.f, pow(dt, 2) / 2.f,
      0, 0, 0, pow(dt, 3) / 2.f, pow(dt, 2), dt,
      0, 0, 0, pow(dt, 2) / 2.f, dt, 1;
  Q = Q * 0.1;  // 乘上加速度方差
  // 测量噪声协方差
  R << 0.0001, 0,0,0,
      0, 0.0001,0,0,
      0,0,0.005,0,
      0,0,0,0.005;
  // 估计误差协方差
  P << .05, .05, .05, 0, 0, 0,
      .05, .05, .05, 0, 0, 0,
      .05, .05, .05, 0, 0, 0,
      0, 0, 0, .05, .05, .05,
      0, 0, 0, .05, .05, .05,
      0, 0, 0, .05, .05, .05;

  kalmanFilter = KalmanFilter{dt, F, H, Q, R, P};

  kalmanFilter.init();

//   odomSensors.imu->set_rotation(gps.get_heading());
}
lemlib::Pose lemlib::getKFPose(bool radians) {
  Eigen::VectorXd state=kalmanFilter.state();
//   lemlib::Pose pose=lemlib::Pose(state(0),state(3),radians?odomPose.theta:radToDeg(odomPose.theta));

  lemlib::Pose pose=lemlib::Pose(meter2Inch(gps->get_status().x),meter2Inch(gps->get_status().y),radians?odomPose.theta:radToDeg(odomPose.theta));

  return pose;
}