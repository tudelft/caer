/*
 * dvs128Calibration.h
 *
 *  Created on: Oct 4, 2016
 *      Author: bas
 */

#ifndef MODULES_OPTICFLOW_DVS128CALIBRATION_H_
#define MODULES_OPTICFLOW_DVS128CALIBRATION_H_

#include "main.h"

#define DVS128_N_PIXELS_X 128
#define DVS128_N_PIXELS_Y 128

/**
 * Camera calibration result container for a DVS128 camera.
 *
 * The struct contains:
 * 	- the principal point coordinates
 * 	- the focal lengths for X and Y
 * 	- pixel coordinate undistortion maps for X and Y
 */
struct dvs128_calibration {
	float principalPointX;
	float principalPointY;
	float focalLengthX;
	float focalLengthY;
	float undistortionMapX[DVS128_N_PIXELS_X][DVS128_N_PIXELS_Y];
	float undistortionMapY[DVS128_N_PIXELS_X][DVS128_N_PIXELS_Y];
};

/**
 * This struct contains the actual calibration result for the DVS128
 * chip with a Kowa TV lens, F1.4 aperture, 4.5mm focal length.
 * The calibration is performed using MATLAB's Camera Calibration Toolbox.
 * Lens distortion is modeled using a 2-parameter radial distortion model.
 *
 * The parameter and map definitions are in the source file for now.
 *
 * Note: for accessing the undistortion maps, use the functions below,
 * which provide bounds checking for X and Y.
 */
extern const struct dvs128_calibration dvs128Calibration;

/**
 * Returns an undistorted pixel X-coordinate based on the
 * dvs128Calibration struct.
 * Includes bounds checking for input X- and Y- addresses.
 *
 * @param x X-address of pixel
 * @param y Y-address of pixel
 * @return Undistorted pixel location
 */
float dvs128GetUndistortedPixelX(uint16_t x, uint16_t y);

/**
 * Returns an undistorted pixel Y-coordinate based on the
 * dvs128Calibration struct.
 * Includes bounds checking for input X- and Y- addresses.
 *
 * @param x X-address of pixel
 * @param y Y-address of pixel
 * @return Undistorted pixel location
 */
float dvs128GetUndistortedPixelY(uint16_t x, uint16_t y);

#endif /* MODULES_OPTICFLOW_DVS128CALIBRATION_H_ */
