#include "PixyTracker.hpp"
#include <sstream>

// Constructor
PixyTracker::PixyTracker () {
	// Allocate buffers for frame data and initialize the Pixy
	image = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
	data = new uint8_t[4 * (320 - 2) * (200 - 2)];
	pixy_init_status = pixy_init();
	initialize_gimbals();
}

// Destructor
PixyTracker::~PixyTracker () {
	pixy_close();
	imaqDispose(image);
	delete data;
	data = nullptr;
}

// startVideo
void PixyTracker::startVideo() {
	// Create a thread to grab the Pixy frames and send them
	// to the Dashboard
	server_thread = new std::thread(&PixyTracker::serveFrames, this, this);
}

// serveFrames
void PixyTracker::serveFrames(PixyTracker *pixy) {
	int response;
	while (true) {
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		std::lock_guard<std::mutex> lock(cmd_mutex);

		pixy_command("stop", END_OUT_ARGS, &response, END_IN_ARGS);
		pixy_command("run", END_OUT_ARGS, &response, END_IN_ARGS);
		pixy->putFrame();
	}
}

// Version
std::string PixyTracker::Version() {
	std::stringstream buffer;
	uint16_t major, minor, build;

	buffer.str("");
	if (pixy_get_firmware_version(&major, &minor, &build) == 0) {
		buffer << major << "." << minor << "." << build;
	} else {
		buffer << "Unknown";
	}
	return buffer.str();
}

// Track
// Track the largest object of the specified signature
// Return the number of objects found and the data for the largest object
int PixyTracker::Track(int signature, Target& target) {
	std::lock_guard<std::mutex> lock(cmd_mutex);

	if (pixy_init_status != 0) {
		printf("Error: pixy_init() [%d] ", pixy_init_status);
		pixy_error(pixy_init_status);
		return pixy_init_status;
	}

	// No new data
	if (!pixy_blocks_are_new()) {
		return 0;
	}

	int target_index = -1;
	int blocks_found = 0;

	// Get blocks from Pixy
	int blocks_copied = pixy_get_blocks(kBLOCK_BUFFER_SIZE, &blocks[0]);

	if (blocks_copied < 0) {
		printf("Error: pixy_get_blocks() [%d] ", blocks_copied);
		pixy_error(blocks_copied);
		return blocks_copied;
	}

	if (blocks_copied > 0) {
		for (int i = 0; i < blocks_copied; ++i) {
			// Ignore blocks that don't match the desired signature
			if (blocks[i].signature != signature) {
				continue;
			}

			// Count the blocks of the matching signature
			++blocks_found;

			// The first block found of the matching signature is the largest -- track it!
			if (target_index < 0) {
				target_index = i;

				target.block = blocks[i];

				// Calculate the difference between the center of Pixy's focus and the target.
				int pan_error  = kPIXY_X_CENTER() - blocks[i].x;
				int tilt_error = blocks[0].y - kPIXY_Y_CENTER();

				// Apply corrections to the pan/tilt with the goal of putting the target in the center of Pixy's focus.
				gimbal_update(&pan, pan_error);
				gimbal_update(&tilt, tilt_error);

				target.pan = pan.position;
				target.tilt = tilt.position;

				int result; // = pixy_rcs_set_position(PIXY_RCS_PAN_CHANNEL, pan.position);
				pixy_command("rcs_setPos",
						INT8(kPIXY_RCS_PAN_CHANNEL),     // mode
						INT16(pan.position),        // xoffset
						END_OUT_ARGS, &result, END_IN_ARGS);

				if (result < 0) {
					printf("Error: pixy_rcs_set_position() [%d] ", result);
					pixy_error(result);
					return result;
				}

				result = pixy_rcs_set_position(kPIXY_RCS_TILT_CHANNEL, tilt.position);
				if (result < 0) {
					printf("Error: pixy_rcs_set_position() [%d] ", result);
					pixy_error(result);
					return result;
				}
			}
		}
	}

	return blocks_found;
}

// putFrame
// Fetch a frame of data from the Pixy, convert it to RGBA, then send it to the
// Dashboard
void PixyTracker::putFrame() {
	uint8_t* videodata;
	int32_t response;
	int32_t fourccc;
	int8_t renderflags;
	uint16_t xwidth;
	uint16_t ywidth;
	uint32_t size;

	int return_value = pixy_command("cam_getFrame",  // String id for remote procedure
			INT8(0x21),     // mode
			INT16(0),       // xoffset
			INT16(0),       // yoffset
			INT16(320),     // width
			INT16(200),     // height
			END_OUT_ARGS,   // separator
			&response,      // pointer to mem address for return value
			&fourccc,
			&renderflags,
			&xwidth,
			&ywidth,
			&size,
			&videodata,     // pointer to mem address for returned frame
			END_IN_ARGS);

	if (return_value == 0) {
		render(renderflags, xwidth, ywidth, size, videodata);
	}
}

// initialize_gimbals
// Initialize the servo control
void PixyTracker::initialize_gimbals()
{
	pan.position           = PIXY_RCS_CENTER_POS;
	pan.previous_error     = 0x80000000L;
	pan.proportional_gain  = kPAN_PROPORTIONAL_GAIN;
	pan.derivative_gain    = kPAN_DERIVATIVE_GAIN;
	tilt.position          = PIXY_RCS_CENTER_POS;
	tilt.previous_error    = 0x80000000L;
	tilt.proportional_gain = kTILT_PROPORTIONAL_GAIN;
	tilt.derivative_gain   = kTILT_DERIVATIVE_GAIN;
}

// gimbal_update
// Determine servo positions
void PixyTracker::gimbal_update(struct Gimbal *  gimbal, int32_t error)
{
	long int velocity;
	int32_t  error_delta;
	int32_t  P_gain;
	int32_t  D_gain;

	if (gimbal->previous_error != 0x80000000L) {
		error_delta = error - gimbal->previous_error;
		P_gain      = gimbal->proportional_gain;
		D_gain      = gimbal->derivative_gain;

		/* Using the proportional and derivative gain for the gimbal,
       calculate the change to the position.  */
		velocity = (error * P_gain + error_delta * D_gain) >> 10;

		gimbal->position += velocity;

		if (gimbal->position > PIXY_RCS_MAX_POS) {
			gimbal->position = PIXY_RCS_MAX_POS;
		} else if (gimbal->position < PIXY_RCS_MIN_POS) {
			gimbal->position = PIXY_RCS_MIN_POS;
		}
	}

	gimbal->previous_error = error;
}

// interpolateBayer
// Interpolate the Bayer CFA data to RBG
void PixyTracker::interpolateBayer(uint16_t width, uint16_t x, uint16_t y, uint8_t *pixel, uint8_t* r, uint8_t* g, uint8_t* b)
{
	if (y&1)
	{
		if (x&1)
		{
			*r = *pixel;
			*g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
			*b = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
		}
		else
		{
			*r = (*(pixel-1)+*(pixel+1))>>1;
			*g = *pixel;
			*b = (*(pixel-width)+*(pixel+width))>>1;
		}
	}
	else
	{
		if (x&1)
		{
			*r = (*(pixel-width)+*(pixel+width))>>1;
			*g = *pixel;
			*b = (*(pixel-1)+*(pixel+1))>>1;
		}
		else
		{
			*r = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
			*g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
			*b = *pixel;
		}
	}
}

// render
// Convert the CFA data from Pixy into an RGBA array
int PixyTracker::render(uint8_t renderFlags, uint16_t width, uint16_t height, uint32_t frameLen, uint8_t *frame)
{
	uint16_t x, y;
	uint8_t r, g, b;

	// skip first line
	frame += width;

	uint m = 0;
	for (y=1; y<height-1; y++)
	{
		frame++;
		for (x=1; x<width-1; x++, frame++)
		{
			interpolateBayer(width, x, y, frame, &r, &g, &b);
			data[m++] = r;
			data[m++] = g;
			data[m++] = b;
			data[m++] = 0xFF; //alpha
		}
		frame++;
	}

	// Create an IMAQ image and send it to the Dashboard
	imaqArrayToImage(image, data, width-2, height-2);
	CameraServer::GetInstance()->SetImage(image);

	return 0;
}
