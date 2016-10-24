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
#include "dvs128Calibration.h"

#define MAX_NUMBER_OF_EVENTS 100
#define FLT_ZERO_EPSILON 1.0E-10f
#define SECONDS_TO_MICROSECONDS 1e6f


void flowAdaptiveComputeFlow(flowEvent e, simple2DBufferLong buffer,
		flowAdaptiveState state) {
	int64_t t = e->timestamp;
	uint8_t x = flowEventGetX(e);
	uint8_t y = flowEventGetY(e);

	// Software based refractory period check
	int64_t Dt = t - simple2DBufferLongGet(buffer, (size_t) x, (size_t) y);
	if (Dt < state->refractoryPeriod) {
		return;
	}
	simple2DBufferLongSet(buffer, x, y, t);

	// Do not compute flow if the flow event rate is too high
	if (state->limitEventRate) {
		if (t - state->lastEventT < 1) {
			return;
		}
		float flowRate = 1e6f/((float) (t - state->lastEventT));
		if (flowRate > state->rateSetpoint) {
			return;
		}
	}

	// Accumulate event timestamps where possible
	size_t n = 1;
	bool isUsed[state->kernelSize];
	int64_t dts[state->kernelSize];
	int64_t dtSum = 0;
	uint32_t i;
	for (i = 0; i < state->kernelSize; i++) {
		isUsed[i] = false; // initialization
		uint8_t xx = (uint8_t) (x + state->dxKernel[i]);
		uint8_t yy = (uint8_t) (y + state->dyKernel[i]);
		if (xx >= buffer->sizeX || yy >= buffer->sizeY)
			continue;
		int64_t dt = t - simple2DBufferLongGet(buffer, (size_t) xx, (size_t)yy);
		if (dt < 0) // causality
			continue;
		isUsed[i] = true;
		dts[i] = dt;
		dtSum += dt;
		n++;
	}

	// Compute mean and reject all events with too large timestamp differences
	int64_t dtMean = dtSum/(int64_t) n;
	if (dtMean > state->dtMax) {
		dtMean = state->dtMax;
	}

	for (i = 0; i < state->kernelSize; i++) {
		if (isUsed[i]) {
			if (dts[i] > dtMean) {
				isUsed[i] = false;
				n--;
			}
		}
	}

	if ((float) n < state->nMin) { // insufficient events - return
		return;
	}

	// Now compute flow statistics
	float sx2 = 0;
	float sy2 = 0;
	float sxy = 0;
	float sxt = 0;
	float syt = 0;

	float xU = dvs128GetUndistortedPixelX(x,y);
	float yU = dvs128GetUndistortedPixelY(x,y);
	float dxs[state->kernelSize];
	float dys[state->kernelSize];

	for (i = 0; i < state->kernelSize; i++) {
		if (isUsed[i]) {
			uint8_t xx = (uint8_t) (x + state->dxKernel[i]);
			uint8_t yy = (uint8_t) (y + state->dyKernel[i]);
			float xxU = dvs128GetUndistortedPixelX(xx,yy);
			float yyU = dvs128GetUndistortedPixelY(xx,yy);
			float dx = xxU - xU;
			float dy = yyU - yU;
			dxs[i] = dx;
			dys[i] = dy;
			int64_t dt = -dts[i];
			sx2 += dx * dx;
			sy2 += dy * dy;
			sxy += dx * dy;
			sxt += dx * (float) dt;
			syt += dy * (float) dt;
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
		uint32_t j;
		size_t nPrevious = n;
		float rejectDistance = sqrtf(a*a+b*b)*state->rejectDistanceFactor;
		for (j = 0; j < state->kernelSize; j++) {
			if (!isUsed[j]) { // already rejected points can be skipped
				continue;
			}
			float dx = dxs[j];
			float dy = dys[j];
			int64_t dt = -dts[j];

			if (fabsf(a*dx + b*dy - (float) dt) > rejectDistance) {
				// remove outlier from sums
				sx2 -= dx * dx;
				sy2 -= dy * dy;
				sxy -= dx * dy;
				sxt -= dx * (float) dt;
				syt -= dy * (float) dt;
				isUsed[j] = false;
				n--;
			}
		}

		if (n == nPrevious) // no change
			break;

		if (n < state->nMin)  // insufficient events
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
	e->xu = xU;
	e->yu = yU;
	e->hasFlow = true;
	int64_t lastFlowDt = t - state->lastEventT;
	if (lastFlowDt > 0) {
		flowAdaptiveUpdateRate(state,lastFlowDt);
	}
	state->lastEventT = t;
}

void flowAdaptiveUpdateRate(flowAdaptiveState state, int64_t lastFlowDt) {
	float lastDtFloat = ((float) lastFlowDt) / SECONDS_TO_MICROSECONDS;
	float rateNew = 1.0f / (lastDtFloat+0.00001f);
	float tFactor = lastDtFloat / state->rateTimeConstant;
	if (tFactor > 1) {
		tFactor = 1;
	}

	state->flowRate += (rateNew - state->flowRate)*tFactor;
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
