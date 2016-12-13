#include "pixy.h"
#include <sstream>
#include <exception>

#define BLOCK_BUFFER_SIZE          25

#define PIXY_X_CENTER              ((PIXY_MAX_X-PIXY_MIN_X)/2)
#define PIXY_Y_CENTER              ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

#define PIXY_RCS_PAN_CHANNEL        0
#define PIXY_RCS_TILT_CHANNEL       1

// PID control parameters
#define PAN_PROPORTIONAL_GAIN     400
#define PAN_DERIVATIVE_GAIN       300
#define TILT_PROPORTIONAL_GAIN    500
#define TILT_DERIVATIVE_GAIN      400

class PixyTracker {
public:
	// Target Info
	struct Target {
		Block block;
		int pan;
		int tilt;
	};

	// Start
	void Start() {
		pixy_init_status = pixy_init();
		initialize_gimbals();
	}

	// Close
	void Close() {
		pixy_close();
	}

	// Version
	std::string Version() {
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
	// Track the largest target of the specified signature
	// signature: signature index to track
	// target:    target info
	// return:    number of targets detected
	int Track(int signature, Target& target) {
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
		int blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, &blocks[0]);

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
					int pan_error  = PIXY_X_CENTER - blocks[i].x;
					int tilt_error = blocks[0].y - PIXY_Y_CENTER;

					// Apply corrections to the pan/tilt with the goal of putting the target in the center of Pixy's focus.
					gimbal_update(&pan, pan_error);
					gimbal_update(&tilt, tilt_error);

					target.pan = pan.position;
					target.tilt = tilt.position;

					int result = pixy_rcs_set_position(PIXY_RCS_PAN_CHANNEL, pan.position);
					if (result < 0) {
						printf("Error: pixy_rcs_set_position() [%d] ", result);
						pixy_error(result);
						return result;
					}

					result = pixy_rcs_set_position(PIXY_RCS_TILT_CHANNEL, tilt.position);
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

private:
	Block 	blocks [BLOCK_BUFFER_SIZE];
	int		pixy_init_status;
	std::stringstream buffer;

	// PID control variables
	struct Gimbal {
		int32_t position;
		int32_t previous_error;
		int32_t proportional_gain;
		int32_t derivative_gain;
	} pan, tilt;

	void initialize_gimbals()
	{
		pan.position           = PIXY_RCS_CENTER_POS;
		pan.previous_error     = 0x80000000L;
		pan.proportional_gain  = PAN_PROPORTIONAL_GAIN;
		pan.derivative_gain    = PAN_DERIVATIVE_GAIN;
		tilt.position          = PIXY_RCS_CENTER_POS;
		tilt.previous_error    = 0x80000000L;
		tilt.proportional_gain = TILT_PROPORTIONAL_GAIN;
		tilt.derivative_gain   = TILT_DERIVATIVE_GAIN;
	}

	void gimbal_update(struct Gimbal *  gimbal, int32_t error)
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
};
