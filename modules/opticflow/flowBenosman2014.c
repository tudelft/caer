/*
 * flowComputation.c
 *
 *  Created on: May 30, 2016
 *      Author: bas
 */

#include "flowBenosman2014.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>
#include "flowEvent.h"

#define MAX_NUMBER_OF_EVENTS 100
#define DBL_ZERO_EPSILON 1.0E-10


void flowBenosman2014(FlowEvent e, FlowEventBuffer buffer,
		FlowBenosman2014Params params) {
	int64_t  t = e->timestamp;
	uint16_t x = caerPolarityEventGetX((caerPolarityEvent) e);
	uint16_t y = caerPolarityEventGetY((caerPolarityEvent) e);;
	bool 	 p = (bool) caerPolarityEventGetPolarity((caerPolarityEvent) e);
	uint16_t xMin = (uint16_t) (x - params.dx/2);
	uint16_t xMax = (uint16_t) (x + params.dx/2);
	uint16_t yMin = (uint16_t) (y - params.dx/2);
	uint16_t yMax = (uint16_t) (y + params.dx/2);

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

	// Accumulate event coordinates in matrix
	int32_t n = 1;
	uint16_t X[MAX_NUMBER_OF_EVENTS];
	uint16_t Y[MAX_NUMBER_OF_EVENTS];
	int64_t T[MAX_NUMBER_OF_EVENTS];
	bool isUsed[MAX_NUMBER_OF_EVENTS];
	X[0] = x;
	Y[0] = y;
	T[0] = t;
	isUsed[0] = true;
	double sx = x;
	double sy = y;
	double st = (double) t;
	double sxy = x*y;
	double sxt = (double) (x*t);
	double syt = (double) (y*t);
	double sx2 = x*x;
	double sy2 = y*y;

	uint16_t xx,yy,i;
	for (xx = xMin; xx < xMax+1; xx++) {
		for (yy = yMin; yy < yMax+1; yy++) {
			for (i = 0; i < (uint16_t) buffer->size; i++) {
				if (n >= MAX_NUMBER_OF_EVENTS) {
					break;
				}
				FlowEvent eB = flowEventBufferRead(buffer,xx,yy,i);
				bool pB = (bool) caerPolarityEventGetPolarity((caerPolarityEvent) eB);
				if (pB != p) {
					continue;
				}
				int64_t tt = eB->timestamp;
				int64_t dt = t - tt;
				if (dt > params.dtMax) {
					break;
				}
				if (dt < params.dtMin) {
					continue;
				}
				// Store event in list
				X[n] = xx;
				Y[n] = yy;
				T[n] = tt;
				isUsed[n] = true;
				n++;
				// Increment sums
				sx += xx;
				sy += yy;
				st += (double) tt;
				sxy += xx*yy;
				sxt += (double) (xx*tt);
				syt += (double) (yy*tt);
				sx2 += xx*xx;
				sy2 += yy*yy;
			}
		}
	}
	// Conditioning
	if (n < 3) { // insufficient events
		return;
	}

	// Compute determinant and check invertibility
	double D = n*sx2*sy2 - sy2*sx*sx + 2*sx*sxy*sy - n*sxy*sxy - sx2*sy*sy;
	if (fabs(D) < DBL_ZERO_EPSILON) { // determinant too small: singular system
	    return;
	}
	// Compute plane parameters
	double a = 1/D*(syt*(n*sxy - sx*sy) - sxt*(- sy*sy + n*sy2) + st*(sx*sy2 - sxy*sy));
	double b = 1/D*(sxt*(n*sxy - sx*sy) - syt*(- sx*sx + n*sx2) - st*(sx*sxy - sx2*sy));
	double d = 1/D*(sxt*(sx*sy2 - sxy*sy) - st*(- sxy*sxy + sx2*sy2) - syt*(sx*sxy - sx2*sy));

	// Iterative outlier rejection
	double eps = 1.0E6;
	int32_t nNew = n;
	while (eps > params.thr1) {
		for (i = 1; i < n; i++) {
			if (!isUsed[i]) { // already rejected points are skipped
				continue;
			}
			if (fabs(a*X[i] + b*Y[i] + (double) T[i] + d) > (double) params.thr2) {
				// remove outlier from sums
				sx2 -= X[i]*X[i];
				sy2 -= Y[i]*Y[i];
				sxy -= X[i]*Y[i];
				sxt -= (double) (X[i]*T[i]);
				syt -= (double) (Y[i]*T[i]);
				sx  -= X[i];
				sy  -= Y[i];
				st  -= (double) T[i];
				isUsed[i] = false;
				nNew--;
			}
		}
		// Conditioning
		if (nNew < 3) { // insufficient events
			return;
		}
		D = nNew*sx2*sy2 - sy2*sx*sx + 2*sx*sxy*sy - nNew*sxy*sxy - sx2*sy*sy;
		if (fabs(D) < DBL_ZERO_EPSILON) { // determinant too small: singular system
			return;
		}
		// Compute plane parameters
		double aNew = 1/D*(syt*(nNew*sxy - sx*sy) - sxt*(- sy*sy + nNew*sy2) + st*(sx*sy2 - sxy*sy));
		double bNew = 1/D*(sxt*(nNew*sxy - sx*sy) - syt*(- sx*sx + nNew*sx2) - st*(sx*sxy - sx2*sy));
		double dNew = 1/D*(sxt*(sx*sy2 - sxy*sy) - st*(- sxy*sxy + sx2*sy2) - syt*(sx*sxy - sx2*sy));

		// Update improvement
		eps = sqrt(pow(aNew-a,2)+pow(bNew-b,2)+pow(dNew-d,2));
		a = aNew;
		b = bNew;
		d = dNew;
	}

	// Compute velocity
	double scaleFactor = -1.0/(a*a+b*b);
	double u = scaleFactor*a;
	double v = scaleFactor*b;

	// Reject if magnitude is too large
	if (1.0/sqrt(u*u+v*v) < params.dtMin) {
		return;
	}

	// Assign flow to event
	e->u = u;
	e->v = v;
	e->hasFlow = true;
}
