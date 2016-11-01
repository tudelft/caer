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
#define N_POINTS_MINIMUM_FIT 3

struct point3d {
	uint8_t xx,yy;
	int64_t dt;
};

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

	// Reject points at edges, since they give rise to erroneous flow
	if (x < 1 || x > buffer->sizeX -2
			|| y < 1 || y > buffer->sizeY -2) {
		return;
	}

	size_t n = 1;
	struct point3d points[state->kernelSize];
	int32_t i;

	// BASED ON 3/10th of events
	// Accumulate event timestamps where possible
	for (i = 0; i < state->kernelSize; i++) {
		uint8_t xx = x + state->dxKernel[i];
		uint8_t yy = y + state->dyKernel[i];
		if (xx >= buffer->sizeX || yy >= buffer->sizeY)
			continue;
		int64_t dt = t - simple2DBufferLongGet(buffer, (size_t) xx, (size_t)yy);
		if (dt < 0 || dt > state->dtMax)
			continue;
		// Sort by timestamp, ascending
		if (n==1) {
			points[0].xx = xx;
			points[0].yy = yy;
			points[0].dt = dt;
		}
		else {
			int32_t j,k;
			bool c = false;
			for (j=0; j < n-1; j++) {
				if (dt < points[j].dt) {
					for (k = n-2; k >= j; k--) {
						points[k+1] = points[k];
					}
					points[j].xx = xx;
					points[j].yy = yy;
					points[j].dt = dt;
					c = true;
					break;
				}
			}
			if (!c) { // append to end
				points[n-1].xx = xx;
				points[n-1].yy = yy;
				points[n-1].dt = dt;
			}
		}
		n++;
	}

	if ((float) n < state->NP) { // insufficient events - return
		return;
	}
	// Cap events to N most recent ones
	n = state->NP;

	// Now compute flow statistics
	float sx2 = 0;
	float sy2 = 0;
	float st2 = 0;
	float sxy = 0;
	float sxt = 0;
	float syt = 0;
	float st = 0;

	float xU = dvs128GetUndistortedPixelX(x,y);
	float yU = dvs128GetUndistortedPixelY(x,y);

	float dxus[state->kernelSize];
	float dyus[state->kernelSize];

	for (i = 0; i < n-1; i++) {
		uint8_t xx = points[i].xx;
		uint8_t yy = points[i].yy;
		float xxU = dvs128GetUndistortedPixelX(xx,yy);
		float yyU = dvs128GetUndistortedPixelY(xx,yy);
		float dx = xxU - xU;
		float dy = yyU - yU;
		dxus[i] = dx;
		dyus[i] = dy;
		float dt = -(float) points[i].dt/SECONDS_TO_MICROSECONDS;
		sx2 += dx*dx;
		sy2 += dy*dy;
		st2 += dt*dt;
		sxy += dx*dy;
		sxt += dx*dt;
		syt += dy*dt;
		st  += dt;
	}

	// Compute determinant and check invertibility
	float D = - sxy*sxy + sx2*sy2;
	if (fabs(D) < FLT_ZERO_EPSILON) { // determinant too small: singular system
		return;
	}
	// Compute plane parameters
	float a = 1/D*(sy2*sxt - sxy*syt);
	float b = 1/D*(sx2*syt - sxy*sxt);

	// Compute R2
	float SSR = st2-a*sxt-b*syt;
	float SST = st2-n*pow(st/n,2);
	float R2 = 1-SSR/SST;

	// If necessary, reject nReject newest points
	if (R2 < state->minR2) {
		bool reject = true;
		size_t nReject;
		for (nReject = 0; nReject < state->nReject; nReject++) {
			size_t i = n - 2;
			if (i < N_POINTS_MINIMUM_FIT) { // fit is no longer possible
				return;
			}
			float dx = dxus[i];
			float dy = dyus[i];
			float dt = -(float) points[i].dt/SECONDS_TO_MICROSECONDS;
			sx2 -= dx*dx;
			sy2 -= dy*dy;
			st2 -= dt*dt;
			sxy -= dx*dy;
			sxt -= dx*dt;
			syt -= dy*dt;
			st  -= dt;
			n--;

			// Compute determinant and check invertibility
			D = - sxy*sxy + sx2*sy2;
			if (fabs(D) < FLT_ZERO_EPSILON) { // determinant too small: singular system
				return;
			}
			// Compute plane parameters
			a = 1/D*(sy2*sxt - sxy*syt);
			b = 1/D*(sx2*syt - sxy*sxt);

			// Compute R2
			SSR = st2-a*sxt-b*syt;
			SST = st2-n*pow(st/n,2);
			R2 = 1-SSR/SST;
			if (R2 >= state->minR2) {
				reject = false;
				break;
			}
		}
		// no improvement seen, return
		if (reject) {
			return;
		}
	}

	// Compute velocity
	float scaleFactor = 1.0f/(a*a + b*b);
	float u = scaleFactor*a;
	float v = scaleFactor*b;
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
	state->NP = (size_t) ceilf(state->nFraction * state->kernelSize);

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
