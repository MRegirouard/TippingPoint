#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "Globals.h"
#include "Settings.h"
#include "api.h"
#include <vector>

/**
 * Encapsulates the autonomous code, helper functions,
 * and color and routine options
 */
namespace Autonomous
{
	/**
	 * The possible options for autonomous routines
	 */
	enum autonOption
	{
		SIDE, // Pick up side goal
		CENTER, // Pick up center goal
		RIGHT_WP, // Get the right neutral goal, then put a ring on the right alliance goal and move it back
		LEFT_WP, // Get the left neutral goal, then put a ring on the left alliance goal
		NONE, // Do nothing
		SKILLS, // Skills routine, same as RIGHT_WP for now
		OBJTRACK, // Object tracking demo / test
		BALANCE, // Balancing demo / test
		COUNT // Used to count the number of autons
	};

	/**
	 * The possible options for autonomous colors
	 */
	enum autonColor
	{
		NO_COLOR = FRONT_VISION_SIGNATURE_INDEX,
		RED = FRONT_RED_VISION_SIGNATURE_INDEX,
		BLUE = FRONT_BLUE_VISION_SIGNATURE_INDEX
	};

	std::vector<const char*> createAutonStrs();
	std::vector<const char*> createAutonColorStrs();

	void frontVisionDrive(const int maxTime = -1, const int sig = 0);
	void rearVisionDrive(const int maxTime = -1);

	void runAutonomous(const autonOption selectedAuton, const autonColor selectedColor = autonColor::NO_COLOR);

	void runSide(const autonColor selectedColor = autonColor::NO_COLOR);
	void runCenter(const autonColor selectedColor = autonColor::NO_COLOR);
	void runRightWP(const autonColor selectedColor = autonColor::NO_COLOR);
	void runLeftWP(const autonColor selectedColor = autonColor::NO_COLOR);
	void runNone(const autonColor selectedColor = autonColor::NO_COLOR);
	void runSkills(const autonColor selectedColor = autonColor::NO_COLOR);
	void runObjTrack(const autonColor selectedColor = autonColor::NO_COLOR);
	void runBalance(const autonColor selectedColor = autonColor::NO_COLOR);
}

#endif // _AUTONOMOUS_H_