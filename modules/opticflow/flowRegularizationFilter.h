/*
 * flowRegularizationFilter.h
 *
 *  Created on: Jun 22, 2016
 *      Author: bas
 */

#ifndef MODULES_OPTICFLOW_FLOWREGULARIZATIONFILTER_H_
#define MODULES_OPTICFLOW_FLOWREGULARIZATIONFILTER_H_

#include "flowEvent.h"
#include "main.h"

typedef struct {
	int64_t dtMax;
	uint16_t dx;
	double maxAngle;
	double maxSpeedFactor;
} FlowRegularizationFilterParams;

/**
 * This filter rejects flow vectors that are not supported by
 * neighboring events with flow in a similar direction and
 * magnitude.
 */
void flowRegularizationFilter(FlowEvent e, FlowEventBuffer buffer,
		FlowRegularizationFilterParams params);


#endif /* MODULES_OPTICFLOW_FLOWREGULARIZATIONFILTER_H_ */
