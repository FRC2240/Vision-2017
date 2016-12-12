#include "WPILib.h"
#include "PixyTracker.hpp"

class Robot: public IterativeRobot {
private:

	PixyTracker pixy;

	void RobotInit()
	{

	}

	void DisabledInit()
	{
		pixy.Close();
	}

	void AutonomousInit()
	{
		pixy.Start();
	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
		pixy.Start();
	}

	void TeleopPeriodic()
	{
		int32_t pan_angle, tilt_angle;
		int blocks_received;

		try {
			blocks_received = pixy.Track(pan_angle, tilt_angle);
		} catch (...) {
			pixy.Reset();
		}

		printf("Pan: %4d, Tilt: %4d\n", pan_angle, tilt_angle);


		// TODO: Other robot tasks...
	}

	void TestPeriodic()
	{
		// Print Pixy firmware version
		std::cout << "Pixy Firmware Version: " << pixy.Version() << std::endl;
	}
};

START_ROBOT_CLASS(Robot);
