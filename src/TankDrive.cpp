#include "TankDrive.h"

/**
 * Construct a new TankDrive with the given motor vectors, wheel diameter,
 * wheel base, and gear ratio
 * @param lMotors The left side motors
 * @param rMotors The right side motors
 * @param wheelDiameter The diameter of the wheels
 * @param wheelBase The distance between the centers of the wheels
 * @param gearRatio The gear ratio of the drivetrain (Motor gears / wheel gears)
 */
TankDrive::TankDrive(const std::vector<pros::Motor*> lMotors, const std::vector<pros::Motor*> rMotors, const float wheelDiameter, const float wheelBase, const float gearRatio)
: motorDegsPerInch(360.0 / (wheelDiameter * M_PI)), motorDegsPerDeg(wheelBase / wheelDiameter)
{
    leftMotors = new MotorGroup(lMotors);
    rightMotors = new MotorGroup(rMotors);
    setAccelLimit(500);
}

/**
 * Sets the deadzone for the drivetrain
 * @param deadzone The deadzone for the drivetrain
 */
void TankDrive::setDeadzone(const int deadzone)
{
    this->deadzone = deadzone;
}

/**
 * Sets the braking mode for all four motors
 * @param mode The braking mode to set
 */
void TankDrive::setBraking(const pros::motor_brake_mode_e mode)
{
    leftMotors->setBraking(mode);
    rightMotors->setBraking(mode);
}

/**
 * Sets the acceleration limit
 * @param maxControlledAccel The maximum acceleration limit, in percentage per second^2
 */
void TankDrive::setAccelLimit(const float maxControlledAccel)
{
    leftMotors->setAccelLimit(maxControlledAccel);
    rightMotors->setAccelLimit(maxControlledAccel);
    accelerationLimit = maxControlledAccel;
}

/**
 * Drives in a "tank drive" fashion, using left and right side speeds. Does
 * not limit acceleration. Uses the "move" call to assign raw voltages, and
 * takes values between -127 and 127
 * @param left The speed for the left side motors of the robot, between -127 and 127
 * @param right The speed for the right side motors of the robot, between -127 and 127
 */
void TankDrive::tankDrive(const int left, const int right)
{
    int newLeft = left, newRight = right;

    if (abs(left) < deadzone * 1.27)
        newLeft = 0;

    if (abs(right) < deadzone * 1.27)
        newRight = 0;

    leftMotors->rawSpin(newLeft);
    rightMotors->rawSpin(newRight);
    leftMotors->forceSetSpin(newLeft / 1.27);
    rightMotors->forceSetSpin(newRight / 1.27);
}

/**
 * Drives in an "arcade drive" fashion, using forward and turning speeds. Does
 * not limit acceleration. Uses the "move" call to assign raw voltages, and
 * takes values between -127 and 127
 * @param speed The forward speed of the robot, between -127 and 127
 * @param turn The turning speed of the robot, between -127 and 127
 */
void TankDrive::arcadeDrive(const int speed, const int turn)
{
    tankDrive(speed + turn, speed - turn);
}

/**
 * Drives in a "tank drive" fashion, using left and right side speeds. Limits
 * acceleration. Uses the "move" call to assign raw voltages, and
 * takes values between -127 and 127
 * @param left The speed for the left side motors of the robot, between -127 and 127
 * @param right The speed for the right side motors of the robot, between -127 and 127
 */
void TankDrive::safeTankDrive(const int left, const int right)
{
    int newLeft = left, newRight = right;

    if (abs(left) < deadzone * 1.27)
        newLeft = 0;

    if (abs(right) < deadzone * 1.27)
        newRight = 0;

    leftMotors->profiledRawSpin(newLeft);
    rightMotors->profiledRawSpin(newRight);
}

/**
 * Drives in an "arcade drive" fashion, using forward and turning speeds. Limits
 * acceleration. Uses the "move" call to assign raw voltages, and
 * takes values between -127 and 127
 * @param speed The forward speed of the robot, between -127 and 127
 * @param turn The turning speed of the robot, between -127 and 127
 */
void TankDrive::safeArcadeDrive(const int speed, const int turn)
{
    safeTankDrive(speed + turn, speed - turn);
}

/**
 * Drives a given distance, and waits for the motion to complete
 * or the timeout to be reached
 * @param distance The distance to drive, in inches
 * @param timeout The timeout for the motion, in milliseconds, -1 for infinite
 * @param velocity The velocity to drive at, a percentage
 */
void TankDrive::waitDriveDist(const float distance, const long timeout, const int velocity)
{
    driveDist(distance, velocity, true, timeout);
}

/**
 * Starts to drive a given distance, and returns immediately
 * @param distance The distance to drive, in inches
 * @param velocity The velocity to drive at, a percentage
 */
void TankDrive::startDriveDist(const float distance, const int velocity)
{
    driveDist(distance, velocity, false, -1);
}

/**
 * Drives a given distance
 * @param distance The distance to drive, in inches
 * @param velocity The velocity to drive at, a percentage
 * @param blocking Whether or not to wait for the motion to complete
 * @param timeout The timeout for the motion, in milliseconds, -1 for infinite
 */
void TankDrive::driveDist(const float distance, const int velocity, const bool blocking, const long timeout)
{
    const float degsToRotate = distance * motorDegsPerInch;

    leftMotors->startRotateDegrees(degsToRotate, velocity);
    rightMotors->startRotateDegrees(degsToRotate, velocity);

    if (blocking)
        waitDriveToTarget(timeout);
}

/**
 * Rotate the robot by given angle, and waits for the motion to complete
 * @param degrees The angle to rotate, in degrees
 * @param timeout The timeout for the motion, in milliseconds, -1 for infinite
 * @param velocity The velocity to rotate at, a percentage
 */
void TankDrive::waitTurn(const float degrees, const long timeout, const int velocity)
{
    turn(degrees, velocity, true, timeout);
}

/**
 * Starts to rotate the robot by given angle, and returns immediately
 * @param degrees The angle to rotate, in degrees
 * @param velocity The velocity to rotate at, a percentage
 */
void TankDrive::startTurn(const float degrees, const int velocity)
{
    turn(degrees, velocity, false, -1);
}

/**
 * Rotate the robot by given angle
 * @param degrees The angle to rotate, in degrees
 * @param velocity The velocity to rotate at, a percentage
 * @param blocking Whether or not to wait for the motion to complete
 * @param timeout The timeout for the motion, in milliseconds, -1 for infinite
 */
void TankDrive::turn(const float degrees, const int velocity, const bool blocking, const long timeout)
{
    const float degsToRotate = degrees * motorDegsPerDeg;

    leftMotors->startRotateDegrees(degsToRotate, velocity);
    rightMotors->startRotateDegrees(-degsToRotate, velocity);

    if (blocking)
        waitDriveToTarget(timeout);
}

/**
 * Waits for the motion to complete
 * @param timeout The timeout for the motion, in milliseconds, -1 for infinite
 */
void TankDrive::waitDriveToTarget(const long timeout)
{
    long startTime = pros::millis();

    while (!reachedTarget() && (timeout == -1 || pros::millis() - startTime < timeout))
    {
        pros::delay(20);
    }
}

/**
 * Determines whether or not the motion has completed
 * @return True if the motion has completed, false otherwise
 */
bool TankDrive::reachedTarget()
{
    return leftMotors->reachedTarget() && rightMotors->reachedTarget();
}

/**
 * Stops the drive, by setting all motor speeds to 0
 */
void TankDrive::stop()
{
    leftMotors->stop();
    rightMotors->stop();
    leftMotors->forceSetSpin(0);
    rightMotors->forceSetSpin(0);
}

/**
 * Gets the average temperature of the motors
 * @return The average temperature of the motors
 */
int TankDrive::getAvgTemp() const
{
    return (leftMotors->getAvgTemp() + rightMotors->getAvgTemp()) / 2;
}

/**
 * Gets the average distance traveled by the motors
 * @return The average distance traveled by the motors, in encoder ticks
 */
int TankDrive::getAvgDist() const
{
    return (leftMotors->getAvgDist() + rightMotors->getAvgDist()) / 2;
}

/**
 * Gets the average distance traveled by the motors
 * @return The average distance traveled by the motors, in inches
 */
float TankDrive::getAvgDistInches() const
{
    return (leftMotors->getGearedPosition() * motorDegsPerInch + rightMotors->getGearedPosition() * motorDegsPerInch) / 2.0;
}

/**
 * Gets the average distance to the target
 * @return The average distance to the target, in encoder ticks
 */
int TankDrive::getAvgTargetDist() const
{
    return (leftMotors->getAvgTargetDist() + rightMotors->getAvgTargetDist()) / 2;
}