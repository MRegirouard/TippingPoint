#include "Arm.h"

/**
 * Construct a new Arm TargetPosition with the given id and angle.
 * Sets the type to ANGLE
 * @param id The id of this target position
 * @param angle The angle of this target position
 */
Arm::TargetPosition::TargetPosition(const int id, const float angle) : id(id), type(ANGLE), angle(angle)
{}

/**
 * Construct a new Arm TargetPosition with the given id and type
 * @param id The id of this target position
 * @param type The type of this target position. If ANGLE is given, 0 is used
 */
Arm::TargetPosition::TargetPosition(const int id, const TargetPositionType type) : id(id), type(type), angle(0)
{}

/**
 * Gets the id of this target position
 * @return The id of this target position
 */
int Arm::TargetPosition::getId() const
{ return id; }

/**
 * Gets the type of this target position
 * @return The type of this target position
 */
Arm::TargetPosition::TargetPositionType Arm::TargetPosition::getType() const
{ return type; }

/**
 * Gets the angle of this target position
 * @return The angle of this target position
 */
float Arm::TargetPosition::getAngle() const
{ return angle; }

/**
 * Construct a new Arm using the given motors, pid, and gear ratio
 * @param motors The motors to use for this Arm
 * @param gearRatio The gear ratio of the motors to the arm (motor teeth / arm teeth)
 */
Arm::Arm(std::vector<pros::Motor*> motors, const float gearRatio) : MotorGroup(motors, gearRatio)
{
	targetPositions = std::vector<const TargetPosition*>();
}

/**
 * Construct a new Arm using the given motors, pid, upper and lower limit switches, and gear ratio
 * @param motors The motors to use for this Arm
 * @param upperLimit The upper limit switch
 * @param lowerLimit The lower limit switch
 * @param gearRatio The gear ratio of the motors to the arm (motor teeth / arm teeth)
 */
Arm::Arm(std::vector<pros::Motor*> motors, const pros::ADIDigitalIn *upperLimit, const pros::ADIDigitalIn *lowerLimit, const float gearRatio) : MotorGroup(motors, gearRatio), upperLimit(upperLimit), lowerLimit(lowerLimit)
{
	targetPositions = std::vector<const TargetPosition*>();
}

/**
 * Raise the arm at the specified speed, between
 * -100 and 100. Uses the "profiledControlledSpin" call
 * to assign velocity values to motors to use their
 * internal PID systems, and limit the acceleration of
 * the arm to the given values. If the arm is at the
 * upper limit, the arm will not move upwards, and if the
 * arm is at the lower limit, the arm will not move downwards
 * @param velocity The percentage speed to raise the arm
 */
void Arm::raise(const int velocity)
{
	if (upperLimit != nullptr && upperLimit->get_value())
	{
		const int newVel = profiledControlledSpinValue(std::min(0, velocity));
		controlledSpin(std::min(0, newVel));
	}
	else if (lowerLimit != nullptr && lowerLimit->get_value())
	{
		const int newVel = profiledControlledSpinValue(std::max(0, velocity));
		controlledSpin(std::max(0, newVel));
	}
	else
		profiledControlledSpin(velocity);
}

/**
 * Lower the arm at the specified speed, between
 * -100 and 100. Uses the "profiledControlledSpin" call
 * to assign velocity values to motors to use their
 * internal PID systems, and limit the acceleration of
 * the arm to the given values. If the arm is at the
 * upper limit, the arm will not move upwards, and if the
 * arm is at the lower limit, the arm will not move downwards
 * @param velocity The percentage speed to lower the arm
 */
void Arm::lower(const int velocity)
{ raise(-velocity); }


/**
 * Sets the acceleration limit for this arm
 * @param maxControlledAccel
 */
void Arm::setAccelLimits(const float maxControlledAccel)
{ maxControlledAcceleration = maxControlledAccel; }

/**
 * Adds the given target position to this arm. Assumes
 * that the target position has a unique id and is not
 * already in the list
 * @param targetPosition The target position to add
 */
void Arm::addTargetPosition(Arm::TargetPosition* targetPosition)
{ targetPositions.push_back(targetPosition); }

/**
 * Adds a target position to this arm using the given angle
 * @param angle The angle for the new TargetPosition
 * @return The id of the new TargetPosition
 */
int Arm::addTargetAngle(const float angle)
{
	int id = targetPositions.size();
	targetPositions.push_back(new TargetPosition(id, angle));
	return id;
}

/**
 * Adds a target position to this arm using the given type
 * @param type The type for the new TargetPosition
 * @return The id of the new TargetPosition
 */
int Arm::addTargetType(const TargetPosition::TargetPositionType type)
{
	int id = targetPositions.size();
	targetPositions.push_back(new TargetPosition(id, type));
	return id;
}

/**
 * Moves the arm to the given target position id
 * @param id The id of the target position to move to
 * @param velocity The velocity to move the arm at
 * @param blocking Whether or not to block until the arm is at the target position
 * @param timeout The movement timeout for the arm, in milliseconds, -1 for infinite
 */
void Arm::moveToTargetPosition(const int id, const int velocity, const bool blocking, const long timeout)
{
	const TargetPosition* targetPosition;

	for (const TargetPosition* targetPos : targetPositions)
	{
		if (targetPos->getId() == id)
		{
			targetPosition = targetPos;
			break;
		}
	}

	moveToTargetPosition(targetPosition, velocity, blocking, timeout);
}

/**
 * Moves the arm to the given target position
 * @param target The target position to move to
 * @param velocity The velocity to move the arm at
 * @param blocking Whether or not to block until the arm is at the target position
 * @param timeout The movement timeout for the arm, in milliseconds, -1 for infinite
 */
void Arm::moveToTargetPosition(const TargetPosition *target, const int velocity, const bool blocking, const long timeout)
{
	if (!blocking)
	{
		pros::Task task([=]()
		{
			moveToTargetPosition(target, velocity, true, timeout);
		}, "Arm Target Position");

		return;
	}

	if (target == nullptr)
		return;

	switch (target->getType())
	{
	case TargetPosition::TargetPositionType::ANGLE:
	{
		waitRotateDegrees(target->getAngle() - getGearedPosition(), velocity, timeout);
		break;
	}
	case TargetPosition::TargetPositionType::TOP:
		if (upperLimit != nullptr)
		{
			const long startTime = pros::millis();

			while (!upperLimit->get_value() && (timeout == -1 || pros::millis() - startTime < timeout))
			{
				raise(abs(velocity));
				pros::delay(20);
			}
		}
		break;
	case TargetPosition::TargetPositionType::BOTTOM:
		if (lowerLimit != nullptr)
		{
			const long startTime = pros::millis();

			while (!lowerLimit->get_value() && (timeout == -1 || pros::millis() - startTime < timeout))
			{
				lower(abs(velocity));
				pros::delay(20);
			}
		}
		break;
	}

	controlledSpin(0);
}