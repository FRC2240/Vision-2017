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
		static int frame = 0;
		++frame;

		int blocks_found = m_pixy->Track(m_signature, m_target);

		if (blocks_found > 0) {
			// Print target info
			if (frame % 50 == 0) {
			    m_pixy->printTargetInfo(m_target);
			}
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
