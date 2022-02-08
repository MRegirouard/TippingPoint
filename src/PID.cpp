#include "PID.h"

/**
 * Construct a new PID controller with the given constants
 * @param kP The proportional constant
 * @param kI The integral constant
 * @param kD The derivative constant
 */
PID::PID(const float kP, const float kI, const float kD)
{
	updateConstants(kP, kI, kD);
}

/**
 * Updates the PID constants to the given values and
 * calls PID::resetTime()
 * @param kP The new proportional constant
 * @param kI The new integral constant
 * @param kD The new derivative constant
 */
void PID::updateConstants(const float kP, const float kI, const float kD)
{
	this->kP = kP;
	this->kI = kI;
	this->kD = kD;
	resetTime();
}

/**
 * Calculates the output of the PID controller with
 * the given error
 * @param error The error to use in the calculation
 * @return float The output of the PID controller
 */
float PID::calcPID(const float error)
{
	const long timeDiff = pros::millis() - prevTime;
	const float dError = (error - prevError) / timeDiff;
	sumError += error * timeDiff;
	prevError = error;
	prevTime = pros::millis();
	return kP * error + kI * sumError + kD * dError;
}

/**
 * Resets the time and error values of the PID controller
 */
void PID::resetTime()
{
	prevError = 0;
	sumError = 0;
	prevTime = pros::millis();
}
