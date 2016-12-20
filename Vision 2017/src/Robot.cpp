#include "WPILib.h"
#include "PixyTracker.hpp"

class Robot: public IterativeRobot {
private:

	PixyTracker *pixy;
	int signature = 1;
	PixyTracker::Target target;

	void RobotInit() {
		// Create the Pixy instance and start streaming the frames
		pixy = new PixyTracker();
		pixy->startVideo();
	}

	void DisabledInit() {
	}

	void AutonomousInit() {
	}

	void AutonomousPeriodic() {
	}

	void TeleopInit() {
	}

	void TeleopPeriodic() {
		int blocks_found = pixy->Track(signature, target);

		if (blocks_found > 0) {
#ifdef DEBUG
			// Print target info
			printf("sig:%2d x:%4d y:%4d width:%4d height:%4d\n",
					target.block.signature,
					target.block.x,
					target.block.y,
					target.block.width,
					target.block.height);
			printf("Pan: %4d, Tilt: %4d\n", target.pan, target.tilt);
#endif
		}

		// TODO: Other robot tasks...
	}

	void TestInit() {
		// Print Pixy firmware version
		std::cout << "Pixy Firmware Version: " << pixy->Version() << std::endl;
	}

	void TestPeriodic() {
	}
};

START_ROBOT_CLASS(Robot);
