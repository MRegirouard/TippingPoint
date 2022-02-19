#ifndef _MOTOR_GROUP_H_
#define _MOTOR_GROUP_H_

#include "api.h"
#include <vector>

#define ENCODER_TICKS_100_RPM 1800 // Number of encoder ticks per revolution for the 100 RPM gearset
#define MOTION_COMPLETE_THRESHOLD 3 // Number of degrees within which a motion is considered complete

/**
 * A group of motors that can be controlled together
 */
class MotorGroup
{
public:
	MotorGroup(const std::vector<pros::Motor*> motors, const float gearRatio = 1);
	MotorGroup(pros::Motor *motor, const float gearRatio = 1);

	void controlledSpin(const int velocity);
	int profiledControlledSpinValue(const int velocity);
	void profiledControlledSpin(const int velocity);
	void rawSpin(const int voltageUnits);
	int profiledRawSpinValue(const int voltageUnits);
	void profiledRawSpin(const int voltageUnits);
	void setBraking(const pros::motor_brake_mode_e_t mode);
	void stop();

	void setAccelLimit(const float maxControlledAcceleration);
	void forceSetSpin(const int velocity);

	void waitRotateDegrees(const int degrees, const int velocity = 50, const long timeout = -1);
	void startRotateDegrees(const int degrees, const int velocity = 50, const long timeout = -1);
	void rotateDegrees(int degrees, int velocity = 50, bool blocking = true, long timeout = -1);
	void waitToReachTarget(const long timeout = -1) const;
	bool reachedTarget() const;

	void setPosition(const int position = 0);

	int getAvgTemp() const;
	int getAvgDist() const;
	int getAvgTargetDist() const;
	float getGearedPosition() const;

protected:
	float maxControlledAcceleration = -1;  // Max acceleration in velocity units per second for profiledControlledSpin

	const std::vector<pros::Motor*> motors;
	const int gearMultiplier;
	const float ticksPerMotionDeg;
	static int getGearMultiplier(const pros::motor_gearset_e_t gearset);
	static int getMotorTicksPerRev(const pros::motor_gearset_e_t gearset);

	long lastProfiledSpinTime = 0;
	int lastControlledSpinVelocity = 0;
};

#endif // _MOTOR_GROUP_H_