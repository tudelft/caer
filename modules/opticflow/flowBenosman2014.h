/*
 * flowComputation.h
 *
 *  Created on: May 30, 2016
 *      Author: bas
 */

#ifndef FLOWBENOSMAN2014_H_
#define FLOWBENOSMAN2014_H_

#include "flowEvent.h"

/**
 * Parameters of the optic flow algorithm.
 */
typedef struct {
	int64_t dtMin;
	int64_t dtMax;
	uint16_t dx;
	double thr1;
	double thr2;
} FlowBenosman2014Params;

/**
 * Implementation of the event-based visual flow algorithm by Benosman et.al. (2014).
 * The algorithm computes optic flow through fitting a local plane to events within
 * the neighborhood of a new event.
 *
 * In this implementation the plane is described by three parameters in the equation:
 *	 	a*x + b*y + t + d = 0
 * The parameters a,b,d are computed through a ordinary least squares estimator:
 * 		A'*A*[a b d]' = A'*t
 * For efficient computation and iterative refinement, the solution is computed manually.
 * To do this, the elements of the matrices (with e.g. sx2 := Sum^{n}_{i=0} x[i]^2 )
 * 		A'*A = [sx2 sxy sx; sxy sy2 sy; sx sy n]
 * and
 *  	A'*t = [sxt; syt; st]
 * are precomputed. Thus, when an outlier is to be rejected, we only decrement the sum values.
 */
void flowBenosman2014(FlowEvent e, FlowEventBuffer buffer,
		FlowBenosman2014Params params);

#endif /* FLOWBENOSMAN2014_H_ */
