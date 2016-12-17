#include "WPILib.h"
#include "PixyTracker.hpp"
#include "PixyServer.h"

class Robot: public IterativeRobot {
private:

	PixyTracker *pixy;

	void RobotInit() {
		pixy = new PixyTracker();
		pixy->Start();
	}

	void DisabledInit() {
	}

	void AutonomousInit() {
	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {
	}

	void TeleopPeriodic()
	{
		int blocks_found;
		int signature = 1;
		PixyTracker::Target target{0};

		blocks_found = pixy->Track(signature, target);

		if (blocks_found > 0) {
			// Print target info
			printf("sig:%2d x:%4d y:%4d width:%4d height:%4d\n",
					target.block.signature,
					target.block.x,
					target.block.y,
					target.block.width,
					target.block.height);
			printf("Pan: %4d, Tilt: %4d\n", target.pan, target.tilt);
		}

		PixyServer::pixyInstance()->pixy_put_frame();

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
