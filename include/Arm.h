#ifndef _ARM_H_
#define _ARM_H_

#include "api.h"
#include <vector>
#include "MotorGroup.h"

/**
 * An arm, which can be raised, lowered, or moved to a target position
 */
class Arm : public MotorGroup
{
public:

	/**
	 * A target position for the arm to move to
	 */
	class TargetPosition
	{
	public:
		/**
		 * The target position type options
		 */
		enum TargetPositionType
		{
			ANGLE, // An angle in degrees
			TOP, // The top of the arm, rotates until the upper limit switch is hit
			BOTTOM, // The bottom of the arm, rotates until the lower limit switch is hit
		};

		TargetPosition(const int id, const float angle);
		TargetPosition(const int id, const TargetPositionType type);

		int getId() const;
		TargetPositionType getType() const;
		float getAngle() const;

	protected:
		const int id;
		const TargetPositionType type;
		const float angle;
	};

	Arm(std::vector<pros::Motor*> motors, const float gearRatio = 1);
	Arm(std::vector<pros::Motor*> motors, const pros::ADIDigitalIn *upperLimit, const pros::ADIDigitalIn *lowerLimit, const float gearRatio = 1);

	void raise(const int velocity = 70);
	void lower(const int velocity = 70);

	void setAccelLimits(const float maxControlledAccel);

	void addTargetPosition(TargetPosition* targetPosition);
	int addTargetAngle(const float angle);
	int addTargetType(const TargetPosition::TargetPositionType type);
	void moveToTargetPosition(const int id, const int velocity = 70, const bool blocking = true, const long timeout = -1);
	void moveToTargetPosition(const TargetPosition *target, const int velocity = 70, const bool blocking = true, const long timeout = -1);

protected:
	const pros::ADIDigitalIn *upperLimit, *lowerLimit;
	std::vector<const TargetPosition*> targetPositions;
};

#endif // _ARM_H_