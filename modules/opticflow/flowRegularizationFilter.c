/*
 * flowRegularizationFilter.c
 *
 *  Created on: Jun 22, 2016
 *      Author: bas
 */

#include "flowRegularizationFilter.h"

#define DEG2RAD M_PI/180

void flowRegularizationFilter(FlowEvent e, FlowEventBuffer buffer,
		FlowRegularizationFilterParams params) {
	if (!e->hasFlow) {	// Only apply filter if the event has flow at all
		return;
	}

	// Extract event properties
	int64_t  t = e->timestamp;
	uint16_t x = caerPolarityEventGetX((caerPolarityEvent) e);
	uint16_t y = caerPolarityEventGetY((caerPolarityEvent) e);
	uint16_t xMin = (uint16_t) (x - params.dx/2);
	uint16_t xMax = (uint16_t) (x + params.dx/2);
	uint16_t yMin = (uint16_t) (y - params.dx/2);
	uint16_t yMax = (uint16_t) (y + params.dx/2);

	double magnitude = sqrt(e->u*e->u + e->v*e->v);
	double angle = atan2(e->v,e->u);
	double rejectMagnitudeDifference = magnitude * params.maxSpeedFactor;
	double rejectAngleDifference = DEG2RAD * params.maxSpeedFactor; // we work with radians
	double magnitudeMean = magnitude;
	double angleMean = angle;

	// Bounds checking
	if (xMin > buffer->sizeX-1) {
		xMin = 0; // In case of uint, cannot be zero.
	}
	if (yMin > buffer->sizeY-1) {
		yMin = 0;
	}
	if (xMax > buffer->sizeX-1) {
		xMax = (uint16_t) (buffer->sizeX-1);
	}
	if (yMax > buffer->sizeY-1) {
		yMax = (uint16_t) (buffer->sizeY-1);
	}

	// Scan in neighborhood for flow vectors
	uint16_t xx,yy,i;
	uint16_t n = 0;
	for (xx = xMin; xx < xMax+1; xx++) {
		for (yy = yMin; yy < yMax+1; yy++) {
			for (i = 0; i < (uint16_t) buffer->size; i++) {
				if (xx == x && yy == y) {
					break;
				}
				FlowEvent eB = flowEventBufferRead(buffer,xx,yy,i);
				if (!eB->hasFlow) {
					continue;
				}
				if (t-eB->timestamp > params.dtMax) {
					break;
				}
				double magnitudeB = sqrt(eB->u*eB->u + eB->v*eB->v);
				double magnitudeDifference = fabs(magnitude - magnitudeB);
				if (magnitudeDifference > rejectMagnitudeDifference) {
					break;
				}
				double angleB = atan2(eB->v, eB->u);
				double num = angle-angleB+M_PI;
				int sig = (num > 0) - (num < 0);
				double angleDifference = fabs(fmod(num*sig,2*M_PI)-M_PI);
				if (angleDifference > rejectAngleDifference) {
					break;
				}
				n++;
				magnitudeMean += magnitudeDifference/n;
				angleMean += angleDifference/n;
				break; // only consider last found flow vector
			}
		}
	}
	if (n == 0) { // no neighbor support
		e->hasFlow = false;
	}
	e->u = cos(angleMean)*magnitudeMean;
	e->v = sin(angleMean)*magnitudeMean;
}
