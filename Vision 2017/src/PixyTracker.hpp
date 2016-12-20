#include "WPILib.h"
#include "pixy.h"
#include <sstream>

#define BLOCK_BUFFER_SIZE          25

#define PIXY_X_CENTER              ((PIXY_MAX_X-PIXY_MIN_X)/2)
#define PIXY_Y_CENTER              ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

#define PIXY_RCS_PAN_CHANNEL        0
#define PIXY_RCS_TILT_CHANNEL       1

// PID control parameters
#define PAN_PROPORTIONAL_GAIN     400
#define PAN_DERIVATIVE_GAIN       600
#define TILT_PROPORTIONAL_GAIN    500
#define TILT_DERIVATIVE_GAIN      700

class PixyTracker {
public:
	// Target Info
	struct Target {
		Block block;
		int pan;
		int tilt;
	};

	PixyTracker();
	~PixyTracker();

	void startVideo();

	// Version
	std::string Version();

	// Track
	// Track the largest target of the specified signature
	// signature: signature index to track
	// target:    target info
	// return:    number of targets detected
	int Track(int signature, Target& target);

private:
	Block 	blocks [BLOCK_BUFFER_SIZE];
	int		pixy_init_status;
	std::stringstream buffer;
	Image *image;
	uint8_t *data;
	std::thread *t1;
	std::mutex g_i_mutex;

	// PID control variables
	struct Gimbal {
		int32_t position;
		int32_t previous_error;
		int32_t proportional_gain;
		int32_t derivative_gain;
	} pan, tilt;

	void initialize_gimbals();
	void gimbal_update(struct Gimbal *  gimbal, int32_t error);
	void serveFrames(PixyTracker *pixy);

	void pixy_put_frame();
	void interpolateBayer(uint16_t width, uint16_t x, uint16_t y, uint8_t *pixel, uint8_t* r, uint8_t* g, uint8_t* b);
	int renderBA81(uint8_t renderFlags, uint16_t width, uint16_t height, uint32_t frameLen, uint8_t *frame);
};
