#include "main.h"
#include "pros/apix.h"
#include "utils.h"
#include "TankDrive.h"
#include "PID.h"
#include "Arm.h"
#include "ControlSystem.h"
#include "Autonomous.h"
#include "Settings.h"
#include "Globals.h"

Controller *controller, *controller2;
ControlSystem *controlSystem;

TankDrive *drive;
ControlSystem::DigitalControl *reverseDriveControl, *balanceControl;
PID *objTrackPID, *balancePID;

Arm *frontArm;
ControlSystem::DigitalControl *frontArmUp, *frontArmDown;
int frontArmUpTarget, frontArmHoldTarget, frontArmDownTarget;

Arm *backArm;
ControlSystem::DigitalControl *backArmUp, *backArmDown;
int backArmUpTarget, backArmHoldTarget, backArmDownTarget;

ADIDigitalOut *clawAirValve;
ControlSystem::DigitalControl *clawClose, *clawOpen;

IMU *imu;
Vision *frontCam, *backCam;

ControlSystem::DigitalControl *brakeToggle;

ControlSystem::BrainStat *frontArmTempsStat, *backArmTempsStat, *driveTempsStat, *autonomousModeStat, *frontVisionStat, *backVisionStat;

std::vector<const char*> autonStrs, autonColors;
Autonomous::autonOption selectedAuton = Autonomous::autonOption::SIDE;
Autonomous::autonColor selectedColor = Autonomous::autonColor::RED;
bool calibrated = false;
bool brakesHolding = true;

/**
 * Update the brain stats
 *
 */
void brainStats()
{
	using namespace std;
	vision_object obj;
	uint32_t lastTime = millis();

	while (true)
	{
		controlSystem->printBrainStats();
		frontArmTempsStat->update({to_string(frontArm->getAvgTemp()), to_string(frontArm->getGearedPosition())});
		backArmTempsStat->update({to_string(backArm->getAvgTemp()), to_string(backArm->getGearedPosition())});
		driveTempsStat->update({to_string(drive->getAvgTemp())});

		obj = frontCam->get_by_sig(0, FRONT_VISION_SIGNATURE_INDEX);
		frontVisionStat->update({to_string(obj.x_middle_coord), to_string(obj.y_middle_coord), to_string(obj.width), to_string(obj.height)});

		obj = backCam->get_by_sig(0, BACK_VISION_SIGNATURE_INDEX);
		backVisionStat->update({to_string(obj.x_middle_coord), to_string(obj.y_middle_coord), to_string(obj.width), to_string(obj.height)});

		Task::delay_until(&lastTime, PRINT_LOOP_DELAY);
	}
}

/**
 * Prints controller stats to the controller
 */
void controllerStats()
{
	using namespace std;
	uint32_t lastTime = pros::millis();

	while (true)
	{
		controlSystem->controllersClear();
		controlSystem->printControllerCompStats();

		string tmp = "D" + to_string(drive->getAvgTemp()) + "*  ";
		tmp += "F" + to_string(frontArm->getAvgTemp()) + "*  ";
		tmp += "B" + to_string(backArm->getAvgTemp()) + "*";
		controlSystem->controllersPrint(1, 0, tmp.c_str());

		if (competition::is_connected())
		{
			if (competition::is_autonomous())
				tmp = "A";
			else
				tmp = "D";

			if (competition::is_disabled())
				tmp += "D ";
			else
				tmp += "E ";
		}
		else
			tmp = "NC ";

		if (selectedColor == Autonomous::autonColor::RED)
			tmp += "R ";
		else if (selectedColor == Autonomous::autonColor::BLUE)
			tmp += "B ";
		else
			tmp += "N ";

		tmp += autonStrs[selectedAuton];
		controlSystem->controllersPrint(2, 0, tmp.c_str());

		Task::delay_until(&lastTime, PRINT_LOOP_DELAY);
	}
}

/**
 * Select autonomous modes, and calibrate arm positions and IMU.
 * Then starts the controller stats printing task
 */
void selectModes()
{
	selectedColor = Autonomous::autonColor(controlSystem->selectionScreen("Color:", autonColors, SELECTION_SCREEN_UP, SELECTION_SCREEN_DOWN, SELECTION_SCREEN_CONFIRM));
	selectedAuton = Autonomous::autonOption(controlSystem->selectionScreen("Autonomous:", autonStrs, SELECTION_SCREEN_UP, SELECTION_SCREEN_DOWN, SELECTION_SCREEN_CONFIRM));
	std::string tmp = autonColors[selectedColor];
	tmp += " ";
	tmp += autonStrs[selectedAuton];
	autonomousModeStat->update(tmp);

	do
	{
		controlSystem->controllersClear();
		controlSystem->controllersPrint(0, 0, "Hold Y to");
		controlSystem->controllersPrint(1, 0, "calibrate...");

		while (!(controller->get_digital(DIGITAL_Y) || controller2->get_digital(DIGITAL_Y)))
			delay(LOOP_DELAY);

		long holdStart = millis();

		controlSystem->controllersPrint(2, 0, "Hold 0.5 sec...  ");

		while ((controller->get_digital(DIGITAL_Y) || controller2->get_digital(DIGITAL_Y)) &&  millis() - holdStart < 500)
			delay(LOOP_DELAY);

		if (millis() - holdStart >= 500)
		{
			controlSystem->controllersPrint(2, 0, "Calibrating... ");
			imu->reset();

			while (imu->is_calibrating())
				delay(LOOP_DELAY);

			calibrated = true;
		}

	} while (!calibrated);

	// Reset arm positions
	frontArm->setPosition(FRONT_ARM_INITIAL_POSITION);
	backArm->setPosition(BACK_ARM_INITIAL_POSITION);

	controlSystem->controllersClear();
	controlSystem->controllersPrint(0, 0, "Calibrated.");
	delay(700);
	controlSystem->controllersClear();

	Task(controllerStats, "Controller Stats");
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	autonStrs = Autonomous::createAutonStrs();
	autonColors = Autonomous::createAutonColorStrs();

	controller = new Controller(E_CONTROLLER_MASTER);
	controller2 = new Controller(E_CONTROLLER_PARTNER);
	controlSystem = new ControlSystem(controller, controller2);

	Motor *frontRight = new Motor(FR_PORT, FR_REVERSED);
	Motor *frontLeft  = new Motor(FL_PORT, FL_REVERSED);
	Motor *backRight  = new Motor(BR_PORT, BR_REVERSED);
	Motor *backLeft   = new Motor(BL_PORT, BL_REVERSED);
	drive = new TankDrive({frontLeft, backLeft}, {frontRight, backRight}, WHEEL_DIAMETER, WHEELBASE, DRIVE_GEAR_RATIO);
	drive->setAccelLimit(DRIVE_ACCEL_LIMIT);
	drive->setBraking(E_MOTOR_BRAKE_COAST);
	drive->setDeadzone(DRIVE_DEADZONE);
	reverseDriveControl = controlSystem->addDigitalControl(REVERSE_DRIVE_CONTROL, ControlSystem::DigitalControl::DigitalControlMode::PRIMARY);
	balanceControl = controlSystem->addDigitalControl(AUTO_BALANCE_CONTROL, ControlSystem::DigitalControl::DigitalControlMode::PRIMARY);
	objTrackPID = new PID(FRONT_OBJ_TRACK_PID);
	balancePID = new PID(BALANCE_PID);

	Motor *frontArmLeft  = new Motor(FRONT_ARM_LEFT_PORT, E_MOTOR_GEARSET_36, FRONT_ARM_LEFT_REVERSED);
	Motor *frontArmRight = new Motor(FRONT_ARM_RIGHT_PORT, E_MOTOR_GEARSET_36, FRONT_ARM_RIGHT_REVERSED);
	ADIDigitalIn *frontArmUpLimit = new ADIDigitalIn(FRONT_ARM_UP_LIMIT_PORT);
	frontArm = new Arm({frontArmLeft, frontArmRight}, frontArmUpLimit, nullptr, FRONT_ARM_GEAR_RATIO);
	frontArm->setBraking(E_MOTOR_BRAKE_COAST);
	frontArm->setAccelLimits(FRONT_ARM_ACCEL_LIMIT);
	frontArmUp = controlSystem->addDigitalControl(FRONT_ARM_UP_CONTROL, ControlSystem::DigitalControl::DigitalControlMode::BOTH);
	frontArmDown = controlSystem->addDigitalControl(FRONT_ARM_DOWN_CONTROL, ControlSystem::DigitalControl::DigitalControlMode::BOTH);
	frontArmUpTarget = frontArm->addTargetType(Arm::TargetPosition::TOP);
	frontArmHoldTarget = frontArm->addTargetAngle(FRONT_ARM_HOLD_POSITION);
	frontArmDownTarget = frontArm->addTargetAngle(FRONT_ARM_DOWN_POSITION);

	Motor *backArmLeft = new Motor(BACK_ARM_LEFT_PORT, E_MOTOR_GEARSET_36, BACK_ARM_LEFT_REVERSED);
	Motor *backArmRight = new Motor(BACK_ARM_RIGHT_PORT, E_MOTOR_GEARSET_36, BACK_ARM_RIGHT_REVERSED);
	ADIDigitalIn *backArmDownLimit = new ADIDigitalIn(BACK_ARM_UP_LIMIT_PORT);
	backArm = new Arm({backArmLeft, backArmRight}, nullptr, backArmDownLimit, BACK_ARM_GEAR_RATIO);
	backArm->setBraking(E_MOTOR_BRAKE_COAST);
	backArm->setAccelLimits(BACK_ARM_ACCEL_LIMIT);
	backArmUp = controlSystem->addDigitalControl(BACK_ARM_UP_CONTROL, ControlSystem::DigitalControl::DigitalControlMode::BOTH);
	backArmDown = controlSystem->addDigitalControl(BACK_ARM_DOWN_CONTROL, ControlSystem::DigitalControl::DigitalControlMode::BOTH);
	backArmUpTarget = backArm->addTargetAngle(BACK_ARM_UP_POSITION);
	backArmHoldTarget = backArm->addTargetAngle(BACK_ARM_HOLD_POSITION);
	backArmDownTarget = backArm->addTargetType(Arm::TargetPosition::BOTTOM);

	imu = new IMU(IMU_PORT);

	frontCam = new Vision(FRONT_VISION_PORT);
	vision_signature frontYellowSig = Vision::signature_from_utility(FRONT_VISION_SIGNATURE_INDEX, FRONT_VISION_SIGNATURE);
	vision_signature frontRedSig = Vision::signature_from_utility(FRONT_RED_VISION_SIGNATURE_INDEX, FRONT_RED_VISION_SIGNATURE);
	vision_signature frontBlueSig = Vision::signature_from_utility(FRONT_BLUE_VISION_SIGNATURE_INDEX, FRONT_BLUE_VISION_SIGNATURE);
	frontCam->set_signature(FRONT_VISION_SIGNATURE_INDEX, &frontYellowSig);
	frontCam->set_signature(FRONT_RED_VISION_SIGNATURE_INDEX, &frontRedSig);
	frontCam->set_signature(FRONT_BLUE_VISION_SIGNATURE_INDEX, &frontBlueSig);
	frontCam->set_zero_point(VISION_ZERO_CENTER);

	backCam = new Vision(BACK_VISION_PORT);
	vision_signature backYellowSig = Vision::signature_from_utility(BACK_VISION_SIGNATURE_INDEX, BACK_VISION_SIGNATURE);
	backCam->set_signature(BACK_VISION_SIGNATURE_INDEX, &backYellowSig);
	backCam->set_zero_point(VISION_ZERO_CENTER);

	clawAirValve = new ADIDigitalOut(CLAW_VALVE_PORT);
	clawAirValve->set_value(CLAW_CLOSE);
	clawClose = controlSystem->addDigitalControl(CLAW_CLOSE_CONTROL, ControlSystem::DigitalControl::DigitalControlMode::BOTH);
	clawOpen = controlSystem->addDigitalControl(CLAW_OPEN_CONTROL, ControlSystem::DigitalControl::DigitalControlMode::BOTH);

	brakeToggle = controlSystem->addDigitalControl(BRAKE_TOGGLE_CONTROL, ControlSystem::DigitalControl::DigitalControlMode::BOTH);

	controlSystem->initBrainStats();
	frontArmTempsStat = new ControlSystem::BrainStat("Front Arm: ", std::vector<std::string> {" *C, ", " *"});
	backArmTempsStat = new ControlSystem::BrainStat("Back Arm: ", std::vector<std::string> {" *C, ", " *"});
	driveTempsStat = new ControlSystem::BrainStat("Drive: ", " *C");
	autonomousModeStat = new ControlSystem::BrainStat("Autonomous: ");
	frontVisionStat = new ControlSystem::BrainStat("Front Obj: (", std::vector<std::string> {" , ", "), ", " by "});
	backVisionStat = new ControlSystem::BrainStat("Back Obj: (", std::vector<std::string> {" , ", "), ", " by "});
	controlSystem->addBrainStat(frontArmTempsStat);
	controlSystem->addBrainStat(backArmTempsStat);
	controlSystem->addBrainStat(driveTempsStat);
	controlSystem->addBrainStat(autonomousModeStat);
	controlSystem->addBrainStat(frontVisionStat);
	controlSystem->addBrainStat(backVisionStat);
	autonomousModeStat->update(autonStrs[selectedAuton]);

	Task selectModesTask(selectModes, "Mode Selector");
	Task printInfo(brainStats, "Brain Stats");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	frontArm->setBraking(E_MOTOR_BRAKE_COAST);
	backArm->setBraking(E_MOTOR_BRAKE_COAST);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
	// Let us know the robot is connected to the field
	controlSystem->controllersRumble(".");
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
	Autonomous::runAutonomous(selectedAuton, selectedColor);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	bool reversedDrive = DRIVE_START_REVERSE_STATE;

	frontArm->setBraking(E_MOTOR_BRAKE_HOLD);
	backArm->setBraking(E_MOTOR_BRAKE_HOLD);

	uint32_t nextBuzz = 75 * 1000; // Buzz when 30 seconds left in the match
	uint32_t lastLoopTime = millis();

	long lastArmAction = millis();

	while (true)
	{
		// Reset PID when auto balancing starts
		if (balanceControl->isNewPress())
			balancePID->resetTime();

		// Auto balance
		if (balanceControl->getValue())
		{
			if (balanceControl->isNewPress())
				balancePID->resetTime();

			const int balanceSpeed = balancePID->calcPID(imu->get_roll());
			drive->safeArcadeDrive(balanceSpeed, 0);
		}
		else // Standard drive
		{
			int forwardSpeed = controller->get_analog(DRIVE_FORWARD);
			int rightSpeed = controller->get_analog(DRIVE_TURN) * DRIVE_TURN_SCALE;
			forwardSpeed += controller->get_analog(DRIVE_SLOW_FORWARD) * DRIVE_SLOW_SCALE;
			rightSpeed += controller->get_analog(DRIVE_SLOW_TURN) * DRIVE_SLOW_TURN_SCALE;

			if (reverseDriveControl->isNewPress())
				reversedDrive = !reversedDrive;

			if (reversedDrive)
				forwardSpeed *= -1;

			drive->safeArcadeDrive(forwardSpeed, rightSpeed);
		}

		// Front Arm
		if (millis() - lastArmAction > FRONT_ARM_DRIVE_TIMEOUT)
		{
			if (frontArmUp->isNewPress())
			{
				frontArm->moveToTargetPosition(frontArmHoldTarget, FRONT_ARM_RAISE_SPEED, false, FRONT_ARM_DRIVE_TIMEOUT);
				lastArmAction = millis();
			}
			else if (frontArmDown->isNewPress())
			{
				frontArm->moveToTargetPosition(frontArmDownTarget, FRONT_ARM_LOWER_SPEED, false, FRONT_ARM_DRIVE_TIMEOUT);
				lastArmAction = millis();
			}
			else
				frontArm->profiledControlledSpin(controller2->get_analog(CONTROLLER2_FRONT_ARM));
		}

		// Claw
		if (clawClose->getValue())
			clawAirValve->set_value(CLAW_CLOSE);
		else if (clawOpen->getValue())
			clawAirValve->set_value(CLAW_OPEN);

		int backArmRaise = controller2->get_analog(CONTROLLER2_BACK_ARM);

		// Back Arm
		if (backArmUp->getValue())
			backArmRaise = BACK_ARM_RAISE_SPEED;
		else if (backArmDown->getValue())
			backArmRaise = -BACK_ARM_LOWER_SPEED;

		backArm->raise(backArmRaise);

		// Toggle motor braking
		if (calibrated && brakeToggle->isNewPress())
		{
			if (brakesHolding)
			{
				frontArm->setBraking(E_MOTOR_BRAKE_COAST);
				backArm->setBraking(E_MOTOR_BRAKE_COAST);
			}
			else
			{
				frontArm->setBraking(E_MOTOR_BRAKE_HOLD);
				backArm->setBraking(E_MOTOR_BRAKE_HOLD);
			}

			brakesHolding = !brakesHolding;
		}

		// Buzz the controller every three minutes so we remember the robot is on
		if (millis() > nextBuzz)
		{
			nextBuzz = millis() + 1000 * 60 * 3;
			controlSystem->controllersRumble(".");
		}

		Task::delay_until(&lastLoopTime, LOOP_DELAY); // Keeps the task running at a consistent rate
	}
}
