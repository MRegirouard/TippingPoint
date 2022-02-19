/**
 * Stores global variables, for use in the main program or autonomous
 */

#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include "api.h"
#include "TankDrive.h"
#include "ControlSystem.h"
#include "PID.h"
#include "Arm.h"

extern pros::Controller *controller, *controller2;
extern ControlSystem *controlSystem;

extern TankDrive *drive;
extern ControlSystem::DigitalControl *reverseDriveControl;
extern PID *objTrackPID;

extern Arm *frontArm;
extern ControlSystem::DigitalControl *frontArmUp, *frontArmDown;
extern int frontArmUpTarget, frontArmHoldTarget, frontArmDownTarget;

extern Arm *backArm;
extern ControlSystem::DigitalControl *backArmUp, *backArmDown;
extern int backArmUpTarget, backArmHoldTarget, backArmDownTarget;

extern pros::ADIDigitalOut *clawAirValve;
extern ControlSystem::DigitalControl *clawClose, *clawOpen;

extern pros::IMU *imu;
extern pros::Vision *frontCam, *backCam;

extern ControlSystem::DigitalControl *brakeToggle;

#endif // _GLOBALS_H_