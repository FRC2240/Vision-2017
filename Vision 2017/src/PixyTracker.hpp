#include "WPILib.h"
#include "pixy.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

class PixyTracker {
public:
	// Target Info
	struct Target {
		Block block;
		int pan;
		int tilt;
		bool is_tracking;
	};

	PixyTracker();
	~PixyTracker();

	// startVideo
	void startVideo();

	// Version
	std::string Version();

	//
	void printTargetInfo(Target& target);

	// Track
	// Track the largest target of the specified signature
	// signature: signature index to track
	// target:    target info
	// return:    number of targets detected
	int Track(int signature, Target& target);

private:

	// Constants
	const static int kBLOCK_BUFFER_SIZE = 10;

	static constexpr int kPIXY_X_CENTER() { return (PIXY_MAX_X-PIXY_MIN_X)/2; }
	static constexpr int kPIXY_Y_CENTER() { return (PIXY_MAX_Y-PIXY_MIN_Y)/2; }

	const int kPIXY_RCS_PAN_CHANNEL  = 0;
	const int kPIXY_RCS_TILT_CHANNEL = 1;

	// PID control parameters
	const int kPAN_PROPORTIONAL_GAIN  = 400;
	const int kPAN_DERIVATIVE_GAIN    = 600;
    const int kTILT_PROPORTIONAL_GAIN = 500;
    const int kTILT_DERIVATIVE_GAIN   = 700;

	Block 	     m_blocks [kBLOCK_BUFFER_SIZE];
	Target		 m_current_target;
	int		     m_pixy_init_status;
	cv::Mat      m_image;
	uint8_t     *m_frame_buffer;
	std::thread *m_server_thread;
	std::mutex   m_cmd_mutex;
	cs::CvSource m_outputStreamStd;

	// PID control variables
	struct Gimbal {
		int32_t position;
		int32_t previous_error;
		int32_t proportional_gain;
		int32_t derivative_gain;
	} m_pan, m_tilt;

	void initialize_gimbals();
	void gimbal_update(struct Gimbal *gimbal, int32_t error);
	void serveFrames(PixyTracker *pixy);

	void putFrame();
	void interpolateBayer(uint16_t width, uint16_t x, uint16_t y, uint8_t *pixel, uint8_t* r, uint8_t* g, uint8_t* b);
	void render(uint8_t renderFlags, uint16_t width, uint16_t height, uint32_t frameLen, uint8_t *frame);
	void sendToDashboard(uint16_t width, uint16_t height);
};
