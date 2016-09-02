/*
 * opticflow.h
 *
 *  Created on: Jun 13, 2016
 *      Author: bas
 */

#ifndef OPTICFLOW_H_
#define OPTICFLOW_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include "modules/opticflow/flowEvent.h"

/**
 * This module produces optic flow estimates from events.
 * It can be used in conjunction with the background activity
 * filter and the visualizer.
 *
 * Currently, only local optic flow values are produced.
 * A future version of this module also computes
 * ego-motion estimates through ventral flow and divergence.
 */
void caerOpticFlowFilter(uint16_t moduleID, caerPolarityEventPacket polarity, flowEventPacket flow);

#endif /* OPTICFLOW_H_ */
