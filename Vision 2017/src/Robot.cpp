#include "WPILib.h"
#include "PixyTracker.hpp"

class Robot: public IterativeRobot {
private:

	PixyTracker *m_pixy;
	int m_signature = 1;
	PixyTracker::Target m_target;

	void RobotInit() {
		// Create the Pixy instance and start streaming the frames
		m_pixy = new PixyTracker();
		m_pixy->startVideo();
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
		int blocks_found = m_pixy->Track(m_signature, m_target);

		if (blocks_found > 0) {
#ifdef DEBUG
			// Print target info
			printf("sig:%2d x:%4d y:%4d width:%4d height:%4d\n",
					m_target.block.signature,
					m_target.block.x,
					m_target.block.y,
					m_target.block.width,
					m_target.block.height);
			printf("Pan: %4d, Tilt: %4d\n", m_target.pan, m_target.tilt);
#endif
			// TODO: Decide what to do about the target...
		}

		// TODO: Other robot tasks...
	}

	void TestInit() {
		// Print Pixy firmware version
		std::cout << "Pixy Firmware Version: " << m_pixy->Version() << std::endl;
	}

	void TestPeriodic() {
	}
};

START_ROBOT_CLASS(Robot);
