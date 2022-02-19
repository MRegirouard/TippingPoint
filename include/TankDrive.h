#ifndef _TANK_DRIVE_H_
#define _TANK_DRIVE_H_

#include "api.h"
#include <vector>
#include "PID.h"
#include "MotorGroup.h"

/**
 * A basic "tank drive" drivetrain with left and right side motors
 */
class TankDrive
{
    public:
        TankDrive(const std::vector<pros::Motor*> lMotors, const std::vector<pros::Motor*> rMotors, const float wheelDiameter = 4.25, const float wheelBase = 12, const float gearRatio = 1.0);

        void setDeadzone(const int deadzone);
        void setBraking(const pros::motor_brake_mode_e_t mode = pros::E_MOTOR_BRAKE_COAST);
        void setAccelLimit(float maxControlledAccel);

        void tankDrive(const int left, const int right);
        void arcadeDrive(const int speed, const int turn);
        void safeTankDrive(const int left, const int right);
        void safeArcadeDrive(const int speed, const int turn);

        void waitDriveDist(const float distance, const long timeout = -1, const int velocity = 70);
        void startDriveDist(const float distance, const int velocity = 70);
        void driveDist(const float distance, const int velocity, const bool blocking, const long timeout);

        void waitTurn(const float degrees, const long timeout = -1, const int velocity = 70);
        void startTurn(const float degrees, const int velocity = 70);
        void turn(const float degrees, const int velocity, const bool blocking, const long timeout);

        void waitDriveToTarget(const long timeout = -1);
        bool reachedTarget();

        void stop();

        int getAvgTemp() const;
        int getAvgDist() const;
        float getAvgDistInches() const;
        int getAvgTargetDist() const;

    protected:
        MotorGroup *leftMotors, *rightMotors;

        int accelerationLimit = -1, deadzone = 0;

        const float motorDegsPerInch; // MotorGroup degrees per forward inch
        const float motorDegsPerDeg; // MotorGroup degrees per rotational degree

};

#endif // _TANK_DRIVE_H_