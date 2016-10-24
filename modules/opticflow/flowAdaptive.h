/*
 * flowComputation.h
 *
 *  Created on: May 30, 2016
 *      Author: bas
 */

#ifndef FLOWADAPTIVE_H_
#define FLOWADAPTIVE_H_

#include "flowEvent.h"


/**
 * States and parameters of the optic flow algorithm.
 */
struct flow_adaptive_state {
	int64_t refractoryPeriod;

	uint8_t dx;
	int64_t dtMax;
	float 	vMax;
	uint32_t rejectIterations;
	float 	rejectDistanceFactor;
	size_t 	nMin;

	bool	limitEventRate;
	float	rateSetpoint;
	float	rateTimeConstant;

	int8_t* dxKernel;
	int8_t* dyKernel;
	size_t 	kernelSize;

	float	flowRate;
	int64_t lastEventT;

	floatArray undistortionMapX;
	floatArray undistortionMapY;
};

typedef struct flow_adaptive_state* flowAdaptiveState;

/**
 * Simplified implementation of the event-based visual flow algorithm by
 * Benosman et.al. (2014).
 * Computes optic flow through fitting a local plane to events within the
 * neighborhood of a new event.
 *
 * In this implementation the plane is described by only two parameters in the
 * equation:
 *
 *	 	a*dx + b*dy + dt = 0
 *
 * The simplification is possible by defining the plane to pass through the new
 * event location, and by considering only relative displacement to the new point
 * in space-time.
 *
 * The parameters a,b,d are computed through an ordinary least squares estimator:
 *
 * 		A'*A*[a b]' = A'*t
 *
 * For efficient computation and iterative refinement, the solution is computed
 * manually.
 * To do this, the elements of the matrices (with e.g. sx2 := Sum^{n}_{i=0} x[i]^2)
 *
 * 		A'*A = [sx2 sxy; sxy sy2];   A'*t = [sxt; syt];
 *
 * are precomputed. Thus, when an outlier is to be rejected, we only need to
 * decrement the sum values instead of recomputing the full plane.
 *
 * @param e New event
 * @param buffer Last timestamp buffer
 * @param state Flow computation state
 */
void flowAdaptiveComputeFlow(flowEvent e, simple2DBufferLong buffer,
		flowAdaptiveState state);

/**
 * Flow output rate is estimated online through a discrete IIR low-pass filter.
 *
 * @param state Flow computation state, containing rate.
 * @param lastFlowDt Time difference between the newest and the previous flow vector.
 */
void flowAdaptiveUpdateRate(flowAdaptiveState state, int64_t lastFlowDt);

/**
 * The search kernels of the flow computation state represent the neighborhood
 * coordinates that are searched by the algorithm. These are initialized dynamically,
 * such that they can be re-initialized when the user changes the neighborhood
 * size parameter through the config server.
 *
 * @param state Flow computation state.
 * @return true if successful.
 */
bool flowAdaptiveInitSearchKernels(flowAdaptiveState state);

/**
 * Free the search kernel memory.
 * @param state Flow computation state.
 */
void flowAdaptiveFreeSearchKernels(flowAdaptiveState state);

#endif /* FLOWADAPTIVE_H_ */
