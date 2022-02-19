#include "MotorGroup.h"

/**
 * Create a new MotorGroup using a vector of Motors. It is assumed
 * that all motors are using the same gearset
 * @param motors The vector of motors to use
 * @param gearRatio The gear ratio of the motors
 */
MotorGroup::MotorGroup(const std::vector<pros::Motor*> motors, const float gearRatio)
: motors(motors), gearMultiplier(getGearMultiplier(motors[0]->get_gearing())), ticksPerMotionDeg(getMotorTicksPerRev(motors[0]->get_gearing()) / 360.0 / gearRatio)
{}

/**
 * Create a new MotorGroup using a single Motor
 * @param motor The motor to use
 * @param gearRatio The gear ratio of the motors
 */
MotorGroup::MotorGroup(pros::Motor *motor, const float gearRatio)
: motors({motor}), gearMultiplier(getGearMultiplier(motor->get_gearing())), ticksPerMotionDeg(getMotorTicksPerRev(motor->get_gearing()) / 360.0 / gearRatio)
{}

/**
 * Spin the motors at the given velocity. This velocity should range from -100
 * to 100, and will be scaled to the appropriate RPM for this MotorGroup.
 * This function calls the Motor::move_velocity() function for each motor,
 * which will work to ensure the motor is spinning at the given velocity
 * using the built-in PID loop
 * @param velocity The velocity to spin the motors at
 */
void MotorGroup::controlledSpin(const int velocity)
{
	for (pros::Motor *m : motors)
		m->move_velocity(velocity * gearMultiplier);
}

/**
 * Gets the acceleration-limited velocity to use for the given velocity
 * @param velocity The velocity to adjust for acceleration limits
 * @return The adjusted velocity
 */
int MotorGroup::profiledControlledSpinValue(const int velocity)
{
	if (maxControlledAcceleration >= 0)
	{
		const long timeDiff = pros::millis() - lastProfiledSpinTime;
		const double maxVelocityDiff = (maxControlledAcceleration * timeDiff) / 1000.0;

		int newVelocity = velocity;

		if (abs(velocity - lastControlledSpinVelocity) > maxVelocityDiff)
		{
			if (velocity > lastControlledSpinVelocity)
				newVelocity = lastControlledSpinVelocity + maxVelocityDiff;
			else
				newVelocity = lastControlledSpinVelocity - maxVelocityDiff;
		}

		lastProfiledSpinTime = pros::millis();
		lastControlledSpinVelocity = newVelocity;

		return newVelocity;
	}
	else
	{
		return velocity;
	}
}

/**
 * Spin the motors at a given velocity, but adjusted for the
 * acceleration limit. This function calls the controllerSpin()
 * function
 * @param velocity The velocity to spin the motors at, after adjusting
 * for acceleration limits
 */
void MotorGroup::profiledControlledSpin(const int velocity)
{
	controlledSpin(profiledControlledSpinValue(velocity));
}

/**
 * Spin the motors at the given voltage. This voltage should range from
 * -127 to 127. This function calls the Motor::move() function for each
 * motor, which will not use the built-in PID loop to accurately
 * control rotational speed
 * @param voltageUnits The voltage to spin the motors at
 */
void MotorGroup::rawSpin(const int voltageUnits)
{
	for (pros::Motor *m : motors)
		m->move(voltageUnits);
}

/**
 * Gets the acceleration-limited voltage to use for the given voltage *
 * @param voltageUnits The voltage to adjust for acceleration limits
 * @return The adjusted voltage
 */
int MotorGroup::profiledRawSpinValue(const int voltageUnits)
{
	if (maxControlledAcceleration >= 0)
	{
		const long timeDiff = pros::millis() - lastProfiledSpinTime;
		const double maxVelocityDiff = (maxControlledAcceleration * 1.27 * timeDiff) / 1000.0;

		int newVolts = voltageUnits;

		if (abs(voltageUnits - lastControlledSpinVelocity * 1.27) > maxVelocityDiff)
		{
			if (voltageUnits > lastControlledSpinVelocity * 1.27)
				newVolts = lastControlledSpinVelocity * 1.27 + maxVelocityDiff;
			else
				newVolts = lastControlledSpinVelocity * 1.27 - maxVelocityDiff;
		}

		lastProfiledSpinTime = pros::millis();
		lastControlledSpinVelocity = newVolts / 1.27;

		return newVolts;
	}
	else
	{
		return voltageUnits;
	}
}

/**
 * Spin the motors at a given voltage, but adjusted for the
 * acceleration limit. This function calls the rawSpin() function
 * @param voltageUnits The voltage to spin the motors at, after adjusting
 * for acceleration limits
 */
void MotorGroup::profiledRawSpin(const int voltageUnits)
{
	rawSpin(profiledRawSpinValue(voltageUnits));
}

/**
 * Sets the braking mode for all motors to the given mode
 * @param mode The braking mode to set the motors to
 */
void MotorGroup::setBraking(const pros::motor_brake_mode_e_t mode)
{
	for (pros::Motor *m : motors)
		m->set_brake_mode(mode);
}

/**
 * Stops all motors, by setting their controlled velocity to 0
 */
void MotorGroup::stop()
{
	controlledSpin(0);
}

/**
 * Sets the acceleration limit for all motors to the given value
 * @param maxControlledAcceleration The acceleration limit to set, in percentage per second^2
 */
void MotorGroup::setAccelLimit(const float maxControlledAcceleration)
{
	this->maxControlledAcceleration = maxControlledAcceleration;
}

/**
 * Forcibly sets the motor spin values, bypassing the acceleration limit.
 * This does not update the acceleration limit, but only updates the
 * internal last spin values
 * @param velocity The velocity to update the spin value to
 */
void MotorGroup::forceSetSpin(const int velocity)
{
	lastControlledSpinVelocity = velocity;
	lastProfiledSpinTime = pros::millis();
}

/**
 * Rotates the motors the given number of degrees at the given velocity,
 * waiting for the motion to complete. If timeout is not -1, the motion
 * will be aborted after the given number of milliseconds
 * @param degrees The number of degrees to rotate the motors
 * @param velocity The velocity to rotate the motors at
 * @param timeout Time to wait for the motion to complete, in milliseconds, -1 for infinite
 */
void MotorGroup::waitRotateDegrees(const int degrees, const int velocity, const long timeout)
{
	rotateDegrees(degrees, velocity, true, timeout);
}

/**
 * Starts to rotate the motors the given number of degrees at the
 * given velocity. Does not wait for the motion to complete and
 * exits immediately. If timeout is not -1, the motion
 * will be aborted after the given number of milliseconds
 * @param degrees The number of degrees to rotate the motors
 * @param velocity The velocity to rotate the motors at
 */
void MotorGroup::startRotateDegrees(const int degrees, const int velocity, const long timeout)
{
	rotateDegrees(degrees, velocity, false, timeout);
}

/**
 * Rotates the motors the given number of degrees at the given velocity.
 * If blocking is true, this function will wait for the motion to complete,
 * otherwise, it will exit immediately. If timeout is not -1, the motion
 * will be aborted after the given number of milliseconds
 * @param degrees The number of degrees to rotate the motors
 * @param velocity The velocity to rotate the motors at
 * @param blocking Whether or not to wait for the motion to complete
 * @param timeout Time to wait for the motion to complete, in milliseconds, -1 for infinite
 */
void MotorGroup::rotateDegrees(const int degrees, const int velocity, const bool blocking, const long timeout)
{
	for (pros::Motor *m : motors)
		m->move_relative(degrees * ticksPerMotionDeg, velocity);

	if (blocking)
		waitToReachTarget(timeout);
}

/**
 * Waits for all motors to reach their target position,
 * or until the given timeout is reached. If timeout is -1,
 * will wait indefinitely
 * @param timeout Time to wait for the motion to complete, in milliseconds, -1 for infinite
 */
void MotorGroup::waitToReachTarget(const long timeout) const
{
	const long startTime = pros::millis();

	while (!reachedTarget() && (timeout < 0 || pros::millis() - startTime < timeout))
		pros::delay(10);
}

/**
 * Returns whether or not motors have reached their target position
 * @return True if motors have reached their target position, false otherwise
 */
bool MotorGroup::reachedTarget() const
{
	return abs(getAvgTargetDist() / ticksPerMotionDeg) <= MOTION_COMPLETE_THRESHOLD;
}

/**
 * Forcibly sets the position of all motor encoders to the given value
 * in degrees
 * @param position The position to set the encoders to, in degrees
 */
void MotorGroup::setPosition(const int position)
{
	for (pros::Motor *m : motors)
		m->set_zero_position(position * ticksPerMotionDeg);
}

/**
 * Gets the average motor temperature
 * @return The average motor temperature, in degrees Celsius
 */
int MotorGroup::getAvgTemp() const
{
	int avgTemp = 0;

	for (pros::Motor *m : motors)
		avgTemp += m->get_temperature();

	return avgTemp / motors.size();
}

/**
 * Gets the average motor position, in encoder ticks
 * @return The average motor position, in encoder ticks
 */
int MotorGroup::getAvgDist() const
{
	int avgDist = 0;

	for (pros::Motor *m : motors)
		avgDist += abs(m->get_position());

	return avgDist / motors.size();
}

/**
 * Gets the average distance to the target position, in encoder ticks
 * @return The average distance to the target position, in encoder ticks
 */
int MotorGroup::getAvgTargetDist() const
{
	int avgTargetDist = 0;

	for (pros::Motor *m : motors)
		avgTargetDist += abs(m->get_target_position() - m->get_position());

	return avgTargetDist / motors.size();
}

/**
 * Gets the average position in degrees of the motors
 * @return The average position in degrees
 */
float MotorGroup::getGearedPosition() const
{
	return getAvgDist() / ticksPerMotionDeg;
}

/**
 * Gets the number of encoder ticks per motor revolution for the given gearset
 * @param gearset The gearset to get the number of ticks per motor revolution for
 * @return The number of encoder ticks per motor revolution
 */
int MotorGroup::getMotorTicksPerRev(const pros::motor_gearset_e_t gearset)
{
	return ENCODER_TICKS_100_RPM / getGearMultiplier(gearset);
}

/**
 * Gets the gear multiplier for the given gearset
 * @param gearset The gearset to get the multiplier for
 * @return 1 for the 100 RPM gearset, 2 for the 200 RPM gearset, and 6 for the 600 RPM gearset
 */
int MotorGroup::getGearMultiplier(const pros::motor_gearset_e_t gearset)
{
	switch (gearset)
	{
	case pros::E_MOTOR_GEARSET_36:
		return 1;
	case pros::E_MOTOR_GEARSET_06:
		return 6;
	case pros::E_MOTOR_GEARSET_18:
	default:
		return 2;
	}
}