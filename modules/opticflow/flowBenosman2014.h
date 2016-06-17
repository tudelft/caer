/*
 * flowComputation.h
 *
 *  Created on: May 30, 2016
 *      Author: bas
 */

#ifndef FLOWBENOSMAN2014_H_
#define FLOWBENOSMAN2014_H_

#include "flowEvent.h"

typedef struct  {
	int64_t dtMin;
	int64_t dtMax;
	uint16_t dx;
	float thr1;
	float thr2;
} FlowBenosman2014Params;

void flowBenosman2014(FlowEvent e, FlowEventBuffer buffer,
		FlowBenosman2014Params params);

#endif /* FLOWBENOSMAN2014_H_ */
