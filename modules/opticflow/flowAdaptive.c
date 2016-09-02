/*
 * flowComputation.c
 *
 *  Created on: May 30, 2016
 *      Author: bas
 */

#include "flowAdaptive.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>
#include "flowEvent.h"

#define MAX_NUMBER_OF_EVENTS 100
#define FLT_ZERO_EPSILON 1.0E-10f
#define SECONDS_TO_MICROSECONDS 1e6f


void flowAdaptiveComputeFlow(flowEvent e, simple2DBufferLong buffer,
		flowAdaptiveState state) {
	int64_t t = e->timestamp;
	uint8_t x = flowEventGetX(e);
	uint8_t y = flowEventGetY(e);

	// Check refractory period
	int64_t Dt = t - simple2DBufferLongGet(buffer, (size_t) x, (size_t) y);

	if (Dt < state->refractoryPeriod) {
		return;
	}

	// Accumulate event coordinates in matrix
	size_t n = 1;
	bool isUsed[state->kernelSize];
	size_t i;
	for (i = 0; i < state->kernelSize; i++) {
		isUsed[i] = false; // initialization
		uint8_t xx = (uint8_t) (x + state->dxKernel[i]);
		uint8_t yy = (uint8_t) (y + state->dyKernel[i]);
		if (xx >= buffer->sizeX || yy >= buffer->sizeY)
			continue;
		int64_t dt = t - simple2DBufferLongGet(buffer, (size_t) xx, (size_t)yy);
		if (dt > state->dtMax)
			continue;
		isUsed[i] = true;
		n++;
	}

	if ((float) n < state->nSetpoint) {
		// insufficient events - return
		return;
	}

	// Now compute flow statistics
	float sx2 = 0;
	float sy2 = 0;
	float sxy = 0;
	float sxt = 0;
	float syt = 0;

	for (i = 0; i < state->kernelSize; i++) {
		if (isUsed[i]) {
			int8_t dx = state->dxKernel[i];
			int8_t dy = state->dyKernel[i];
			int64_t dt = -t + simple2DBufferLongGet(buffer,
					(size_t) (x+dx), (size_t) (y+dy));
			sx2 += (float) (dx*dx);
			sy2 += (float) (dy*dy);
			sxy += (float) (dx*dy);
			sxt += (float) (dx*dt);
			syt += (float) (dy*dt);
		}
	}

	// Compute determinant and check invertibility
	float D = - sxy*sxy + sx2*sy2;
	if (fabsf(D) < FLT_ZERO_EPSILON) { // determinant too small: singular system
		return;
	}
	// Compute plane parameters
	float a = 1/D*(sy2*sxt - sxy*syt);
	float b = 1/D*(sx2*syt - sxy*sxt);

	// Iterative outlier rejection
	for (i = 0; i < state->rejectIterations; i++) {
		size_t j;
		size_t nPrevious = n;
		for (j = 0; j < state->kernelSize; j++) {
			if (!isUsed[j]) { // already rejected points are skipped
				continue;
			}
			int8_t dx = state->dxKernel[j];
			int8_t dy = state->dyKernel[j];
			int64_t dt = -t + simple2DBufferLongGet(buffer,
					(size_t) (x+dx), (size_t) (y+dy));

			if (fabsf(a*dx + b*dy - (float) dt) > state->rejectDistance) {
				// remove outlier from sums
				sx2 -= (float)(dx*dx);
				sy2 -= (float)(dy*dy);
				sxy -= (float)(dx*dy);
				sxt -= (float)(dx*dt);
				syt -= (float)(dy*dt);
				isUsed[j] = false;
				n--;
			}
		}
		if (n == nPrevious) // no change
			break;

		if (n < state->nSetpoint)  // insufficient events
			return;

		// Recompute determinant
		D = - sxy*sxy + sx2*sy2;
		if (fabsf(D) < FLT_ZERO_EPSILON) { // determinant too small: singular system
			return;
		}
		// Update plane parameters
		a = 1/D*(sy2*sxt - sxy*syt);
		b = 1/D*(sx2*syt - sxy*sxt);
	}

	// Compute velocity
	float scaleFactor = 1.0f/(a*a + b*b);
	float u = scaleFactor*a*SECONDS_TO_MICROSECONDS;
	float v = scaleFactor*b*SECONDS_TO_MICROSECONDS;
	// Check for NaN value
	if (isnan(u) || isnan(v)) {
		return;
	}

	// Reject if magnitude is too large
	if (sqrtf(u*u+v*v) > state->vMax) {
		return;
	}

	// Assign flow to event in pixels per second (instead of pix/us)
	e->u = u;
	e->v = v;
	e->hasFlow = true;
}

void flowAdaptiveUpdateSetpoint(flowAdaptiveState state, int64_t lastFlowDt) {
	float lastDtFloat = ((float) lastFlowDt) / SECONDS_TO_MICROSECONDS;
	float rateNew = 1.0f / (lastDtFloat+0.00001f);
	float tFactor = lastDtFloat / state->adaptiveTimeConstant;
	if (tFactor > 1) {
		tFactor = 1;
	}

	state->flowRate += (rateNew - state->flowRate)*tFactor;

	if (state->flowRate > state->adaptiveRateSetpoint) {
		state->nSetpoint *= state->adaptiveNGain;
		if (state->nSetpoint > state->kernelSize)
			state->nSetpoint = (float) state->kernelSize;
	}
	if (state->flowRate < state->adaptiveRateSetpoint) {
		state->nSetpoint /= state->adaptiveNGain;
		if (state->nSetpoint < state->adaptiveNMin)
			state->nSetpoint = (float) state->adaptiveNMin;
	}
}

bool flowAdaptiveInitSearchKernels(flowAdaptiveState state) {
	size_t window = (size_t) state->dx * 2 + 1;
	state->kernelSize = window*window - 1;
	state->dxKernel = malloc(state->kernelSize * sizeof(int8_t));
	state->dyKernel = malloc(state->kernelSize * sizeof(int8_t));

	int8_t x,y;
	size_t n = 0;
	for (x = (int8_t) -((int8_t) state->dx); x <= (int8_t) state->dx; x++) {
		for (y = (int8_t) -((int8_t) state->dx); y <= (int8_t) state->dx; y++) {
			if (x == 0 && y == 0)
				continue;
			state->dxKernel[n] = x;
			state->dyKernel[n] = y;
			n++;
			if (n > state->kernelSize)
				return (false);
		}
	}
	if (n < state->kernelSize)
		return (false);
	return (true);
}

void flowAdaptiveFreeSearchKernels(flowAdaptiveState state) {
	free(state->dxKernel);
	free(state->dyKernel);
}
