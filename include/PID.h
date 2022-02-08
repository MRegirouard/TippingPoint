#ifndef _PID_H_
#define _PID_H_

#include "api.h"

/**
 * A basic PID controller
 */
class PID
{
public:
	PID(const float kP, const float kI = 0, const float kD = 0);
	void updateConstants(const float kP, const float kI = 0, const float kD = 0);
	float calcPID(const float error);
	void resetTime();

protected:
	/**
	 * The proportional constant
	 */
	float kP;

	/**
	 * The integral constant
	 */
	float kI;

	/**
	 * The derivative constant
	 */
	float kD;

	/**
	 * The error the last time the PID was run
	 */
	float prevError;

	/**
	 * The sum of all previous error, with respect to time
	 */
	float sumError;

	/**
	 * The previous time the PID controller was updated
	 */
	long prevTime;
};

#endif // _PID_H_
