#include "Autonomous.h"

/**
 * Creates a vector of C-strings for the autonomous options
 * @return The vector of C-strings containing the autonomous options
 */
std::vector<const char*> Autonomous::createAutonStrs()
{
	std::vector<const char*> autonStrs = std::vector<const char*>(COUNT - 1);

	autonStrs[SIDE] = "Side";
	autonStrs[CENTER] = "Center";
	autonStrs[RIGHT_WP] = "RWP";
	autonStrs[LEFT_WP] = "LWP";
	autonStrs[NONE] = "None";
	autonStrs[SKILLS] = "Skills";
	autonStrs[OBJTRACK] = "Obj Track";
	autonStrs[BALANCE] = "Balance";

	return autonStrs;
}

/**
 * Creates a vector of C-strings for the autonomous color options
 * @return The vector of C-strings containing the autonomous color options
 */
std::vector<const char*> Autonomous::createAutonColorStrs()
{
	std::vector<const char*> autonColorStrs = std::vector<const char*>(3);

	autonColorStrs[NO_COLOR] = "None";
	autonColorStrs[RED] = "Red";
	autonColorStrs[BLUE] = "Blue";

	return autonColorStrs;
}

/**
 * Runs the given autonomous routine for the given color
 * @param selectedAuton The autonomous routine to run
 * @param selectedColor The color to run the routine for
 */
void Autonomous::runAutonomous(const autonOption selectedAuton, const autonColor selectedColor)
{
	frontArm->setBraking(pros::E_MOTOR_BRAKE_HOLD);

	switch (selectedAuton)
	{
	case SIDE:
		runSide(selectedColor);
		break;
	case CENTER:
		runCenter(selectedColor);
		break;
	case RIGHT_WP:
		runRightWP(selectedColor);
		break;
	case LEFT_WP:
		runLeftWP(selectedColor);
		break;
	case NONE:
		runNone(selectedColor);
		break;
	case SKILLS:
		runSkills(selectedColor);
		break;
	case OBJTRACK:
		runObjTrack(selectedColor);
		break;
	case BALANCE:
		runBalance(selectedColor);
		break;
	default:
		break;
	}

	frontArm->setBraking(pros::E_MOTOR_BRAKE_COAST);
	backArm->setBraking(pros::E_MOTOR_BRAKE_COAST);
}

/**
 * Drives forwards, using the front vision sensor and
 * PID loop to navigate towards the given vision signature
 * @param maxTime The time to drive forwards, in milliseconds, -1 for infinite
 */
void Autonomous::frontVisionDrive(const int maxTime, const int sig)
{
	pros::vision_object_s_t obj;
	const long startTime = pros::millis();
	objTrackPID->resetTime();

	while (maxTime == -1 || pros::millis() - startTime < maxTime)
	{
		try // Catch errors in case obj is null or something
		{
			obj = frontCam->get_by_sig(0, sig);

			if (obj.width < 15 || obj.height < 10)
				drive->arcadeDrive(90, 0);
			else
				drive->arcadeDrive(90, objTrackPID->calcPID(obj.x_middle_coord));
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << std::endl;
		}

		pros::delay(LOOP_DELAY);
	}

	drive->arcadeDrive(0, 0);
}

/**
 * Drives backwards, using the rear vision sensor and
 * PID loop to navigate towards the neutral mobile goal
 * @param maxTime The time to drive backwards, in milliseconds, -1 for infinite
 */
void Autonomous::rearVisionDrive(const int maxTime)
{
	pros::vision_object_s_t obj;
	const long startTime = pros::millis();
	objTrackPID->resetTime();

	while (maxTime == -1 || pros::millis() - startTime < maxTime)
	{
		try // Catch errors in case obj is null or something
		{
			obj = backCam->get_by_sig(0, BACK_VISION_SIGNATURE_INDEX);

			if (obj.width < 15 || obj.height < 10)
				drive->arcadeDrive(-90, 0);
			else
				drive->arcadeDrive(-90, objTrackPID->calcPID(obj.x_middle_coord));
		}
		catch(const std::exception& e)
		{
			std::cerr << "Object tracking error: " << e.what() << std::endl;
		}

		pros::delay(LOOP_DELAY);
	}

	drive->arcadeDrive(0, 0);
}

/**
 * Lowers the arm, then uses the vision sensor to drive to
 * the side goal. Then, picks up the side goal, then drives
 * backward to the alliance side. Then, puts the arm down,
 * and drives backwards to score the rings on the side goal
 * @param selectedColor Unused, but kept for consistency
 */
void Autonomous::runSide(const autonColor selectedColor)
{
	// Prepare arm
	frontArm->moveToTargetPosition(frontArmDownTarget, 100, true, 1200);
	clawAirValve->set_value(CLAW_OPEN);

	// Drive to the side goal
	frontVisionDrive(2000);

	// Pick up the side goal
	frontArm->setBraking(pros::E_MOTOR_BRAKE_HOLD);
	frontArm->moveToTargetPosition(frontArmHoldTarget, 100, false, 1000);
	pros::delay(700); // Make sure the arm is at least off the ground

	// Drive to the home position
	drive->waitDriveDist(-50, 2500, 100);

	// Lower the arm
	frontArm->moveToTargetPosition(frontArmDownTarget, 100, true, 1000);

	// Back up to score rings on goal
	drive->waitDriveDist(-12, 1000, 100);
}

/**
 * Lowers the arm, then drives forward an amount. Then,
 * uses the vision sensor to drive the rest of the way to
 * the center goal. Picks up the center goal, then drives
 * backward to the alliance side. Then, puts the arm down
 * and drives backward to score the rings on the center goal
 * @param selectedColor Unused, but kept for consistency
 */
void Autonomous::runCenter(const autonColor selectedColor)
{
	// Prepare arm
	frontArm->moveToTargetPosition(frontArmDownTarget, 100, true, 1000);
	clawAirValve->set_value(CLAW_OPEN);

	drive->waitDriveDist(30, 2000, 100);

	// Drive to the middle goal
	frontVisionDrive(1500);

	// Pick up the middle goal
	frontArm->setBraking(pros::E_MOTOR_BRAKE_HOLD);
	frontArm->moveToTargetPosition(frontArmHoldTarget, 100, false, 1000);
	pros::delay(700); // Make sure the arm is at least off the ground

	// Drive to the home position
	drive->waitDriveDist(-50, 2500, 100);

	// Lower the arm
	frontArm->moveToTargetPosition(frontArmDownTarget, 100, true, 1000);

	// Back up to score rings on goal
	drive->waitDriveDist(-12, 1000, 100);
}

/**
 * Drives backwards to fetch the side neutral mobile goal,
 * then returns and turns to face the alliance goal.
 * Releases the neutral mobile goal and drives forward,
 * picks up the alliance goal, and drives backwards to
 * move it off of the diagonal line. Then, drives backwards
 * to score the rings on the alliance goal
 * @param selectedColor The color of the alliance goal
 */
void Autonomous::runRightWP(const autonColor selectedColor)
{
	// Open claw
	clawAirValve->set_value(CLAW_OPEN);

	// Drive to the right goal
	rearVisionDrive(1400);

	// Close the claw and wait
	clawAirValve->set_value(CLAW_CLOSE);
	pros::delay(150);

	// Drive back to the alliance side
	drive->waitDriveDist(50, 1500, 100);

	// Lower the arm
	frontArm->moveToTargetPosition(frontArmDownTarget, 100, false, 1200);

	// Turn towards the alliance goal
	drive->waitTurn(-135, 2500, 70);

	// Release the neutral goal
	clawAirValve->set_value(CLAW_OPEN);

	// Drive to the alliance goal
	frontVisionDrive(1500, selectedColor);

	// Pick up the alliance goal
	frontArm->moveToTargetPosition(frontArmHoldTarget, 100, true, 1500);

	// Back up off the diagonal line
	drive->waitDriveDist(-20, 1000, 100);

	// Lower the alliance goal
	frontArm->moveToTargetPosition(frontArmDownTarget, 100, true, 1500);

	// Back up to score rings on goal
	drive->waitDriveDist(-12, 1000, 100);
}

/**
 * Run the left win point autonomous
 * @param selectedColor The color of the alliance goal
 */
void Autonomous::runLeftWP(const autonColor selectedColor)
{
	// Open claw
	clawAirValve->set_value(CLAW_OPEN);

	// Drive to the left goal
	rearVisionDrive(1500);

	// Close the claw and wait
	clawAirValve->set_value(CLAW_CLOSE);
	pros::delay(150);

	// Drive back to the alliance side
	drive->waitDriveDist(32, 2000, 100);

	// Drop the neutral goal
	clawAirValve->set_value(CLAW_OPEN);

	// Drive towards the platform and then back up
	drive->waitDriveDist(40, 1500, 100);
	drive->waitDriveDist(-5, 500, 100);

	// Turn towards the platform
	drive->waitTurn(-95, 2500, 70);

	// Back up
	drive->waitDriveDist(-10, 1000, 100);

	// Lower the arm
	frontArm->moveToTargetPosition(frontArmDownTarget, 100, true, 2000);

	// Drive forward into the alliance goal
	frontVisionDrive(1500, selectedColor);

	// Back up to score rings on goal
	drive->waitDriveDist(-20, 1000, 100);
}

/**
 * Do nothing
 * @param selectedColor Unused, but kept for consistency
 */
void Autonomous::runNone(const autonColor selectedColor)
{}

/**
 * Run the right win point autonomous
 * @param selectedColor The color of the alliance goal, passed
 * directly to the runRightWP function
 */
void Autonomous::runSkills(const autonColor selectedColor)
{ runRightWP(selectedColor); }

/**
 * Run the front camera object tracking demo / test. Uses
 * a PID loop to rotate the robot to keep a yellow mobile
 * goal in the center of the vision sensor's view
 * @param selectedColor The color of the goal to track
 */
void Autonomous::runObjTrack(const autonColor selectedColor)
{
	PID pid(FRONT_OBJ_TRACK_PID);

	int turnSpeed;

	while (true)
	{
		pros::vision_object_s_t obj = frontCam->get_by_sig(0, selectedColor);

		turnSpeed = 30; // Default to spinning around to search for object

		try // Catch errors in case obj is null or something
		{
			if (obj.width > 20 && obj.height > 15)
			{
				turnSpeed = pid.calcPID(obj.x_middle_coord);
				drive->arcadeDrive(0, turnSpeed);
				pros::delay(LOOP_DELAY);
			}
		}
		catch (const std::exception& e)
		{
			std::cerr << "Object tracking error: " << e.what() << std::endl;
		}
	}
}

/**
 * Run the balance demo / test. Uses a PID loop
 * to keep the robot balanced on the platform
 * @param selectedColor Unused but kept for consistency
 */
void Autonomous::runBalance(const autonColor selectedColor)
{
	PID balancePID(BALANCE_PID);

	while (true)
	{
		drive->arcadeDrive(balancePID.calcPID(imu->get_roll()), 0);
		pros::delay(LOOP_DELAY);
	}
}