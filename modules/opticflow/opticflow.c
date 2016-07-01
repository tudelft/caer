/*
 * opticflow.c
 *
 *  Created on: Jun 13, 2016
 *      Author: bas
 */

#include "opticflow.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/portable_time.h"
#include "flowEvent.h"
#include "flowBenosman2014.h"
#include "flowRegularizationFilter.h"
#include "uart.h"

#define FLOW_BUFFER_SIZE 3
#define DVS128_LOCAL_FLOW_TO_VENTRAL_FLOW 1e6/115.0

char* UART_PORT = "/dev/ttyS2";

bool tested = false;

struct OpticFlowFilter_state {
	FlowEventBuffer buffer;
	FlowBenosman2014Params flowParams;
	FlowRegularizationFilterParams filterParams;
	bool enableFlowRegularization;
	int64_t refractoryPeriod;
	int8_t subSampleBy;
	double wx, wy;
	struct timespec timeInit;
	int64_t timeInitEvent;
	bool timeSet;
	bool uartConnectionReady;
};

typedef struct OpticFlowFilter_state *OpticFlowFilterState;

static bool caerOpticFlowFilterInit(caerModuleData moduleData);
static void caerOpticFlowFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerOpticFlowFilterConfig(caerModuleData moduleData);
static void caerOpticFlowFilterExit(caerModuleData moduleData);
static bool allocateBuffer(OpticFlowFilterState state, int16_t sourceID);
static int64_t computeTimeDelay(OpticFlowFilterState state, int64_t timeEvent);

static struct caer_module_functions caerOpticFlowFilterFunctions = { .moduleInit =
	&caerOpticFlowFilterInit, .moduleRun = &caerOpticFlowFilterRun, .moduleConfig =
	&caerOpticFlowFilterConfig, .moduleExit = &caerOpticFlowFilterExit };

void caerOpticFlowFilter(uint16_t moduleID, FlowEventPacket flow) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "OpticFlow");
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerOpticFlowFilterFunctions, moduleData, sizeof(struct OpticFlowFilter_state), 1, flow);
}

static bool caerOpticFlowFilterInit(caerModuleData moduleData) {
	sshsNodePutLongIfAbsent(moduleData->moduleNode, "refractoryPeriod", 10000);

	sshsNodePutLongIfAbsent(moduleData->moduleNode, "flow_dtMin", 3);
	sshsNodePutLongIfAbsent(moduleData->moduleNode, "flow_dtMax", 300000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode,  "flow_dx", 3);
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "flow_thr1", 1E5);
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "flow_thr2", 5E3);

	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "filter_enable",true);
	sshsNodePutLongIfAbsent(moduleData->moduleNode, "filter_dtMax", 300000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode,  "filter_dx", 3);
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "filter_dMag", 1.0);
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "filter_dAngle", 20);

	sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);

	OpticFlowFilterState state = moduleData->moduleState;

	state->refractoryPeriod = sshsNodeGetLong(moduleData->moduleNode, "refractoryPeriod");

	state->flowParams.dtMin = sshsNodeGetLong(moduleData->moduleNode, "flow_dtMin");
	state->flowParams.dtMax = sshsNodeGetLong(moduleData->moduleNode, "flow_dtMax");
	state->flowParams.dx 	= (uint16_t) sshsNodeGetInt(moduleData->moduleNode, "flow_dx");
	state->flowParams.thr1  = sshsNodeGetDouble(moduleData->moduleNode, "flow_thr1");
	state->flowParams.thr2  = sshsNodeGetDouble(moduleData->moduleNode, "flow_thr2");

	state->enableFlowRegularization = sshsNodeGetBool(moduleData->moduleNode, "filter_enable");
	state->filterParams.dtMax = sshsNodeGetLong(moduleData->moduleNode, "filter_dtMax");
	state->filterParams.dx = (uint16_t) sshsNodeGetInt(moduleData->moduleNode, "filter_dx");
	state->filterParams.maxSpeedFactor = sshsNodeGetDouble(moduleData->moduleNode, "filter_dMag");
	state->filterParams.maxAngle = sshsNodeGetDouble(moduleData->moduleNode, "filter_dAngle");

	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

	state->wx = 0;
	state->wy = 0;

	state->timeSet = false;

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Init UART communication
	int uartErrno = uart_open(UART_PORT);
	if (uartErrno) {
		caerLog(CAER_LOG_ALERT, "UART","Unable to identify serial communication, errno=%i.",uartErrno);
	}
	else {
		state->uartConnectionReady = true;
	}

	// Nothing that can fail here.
	return (true);
}

static void caerOpticFlowFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	FlowEventPacket flow = va_arg(args, FlowEventPacket);

	// Only process packets with content.
	if (flow == NULL) {
		return;
	}

	OpticFlowFilterState state = moduleData->moduleState;

	// If the map is not allocated yet, do it.
	if (state->buffer == NULL) {
		if (!allocateBuffer(state, caerEventPacketHeaderGetEventSource((caerEventPacketHeader) flow))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for timestampMap.");
			return;
		}
	}

	int64_t delay = 0;

	// Iterate over events and filter out ones that are not supported by other
	// events within a certain region in the specified timeframe.
	for (int32_t i = 0; i < caerEventPacketHeaderGetEventNumber((caerEventPacketHeader) flow); i++) {
		FlowEvent e = flowEventPacketGetEvent(flow,i);
		if (!caerPolarityEventIsValid((caerPolarityEvent) e)) { continue; } // Skip invalid events.

		// Refractory period
		uint16_t x = caerPolarityEventGetX((caerPolarityEvent) e);
		uint16_t y = caerPolarityEventGetY((caerPolarityEvent) e);
		FlowEvent eB = flowEventBufferRead(state->buffer,x,y,0);
		if (e->timestamp - eB->timestamp < state->refractoryPeriod) {
			caerPolarityEventInvalidate((caerPolarityEvent) e,
					(caerPolarityEventPacket) flow);
		}

		// Compute optic flow using events in buffer
		flowBenosman2014(e,state->buffer,state->flowParams);

		// Add event to buffer
		flowEventBufferAdd(e,state->buffer);

		// Apply flow regularization filter
		if (state->enableFlowRegularization) {
			flowRegularizationFilter(e,state->buffer,state->filterParams);
		}

		// For now, count events in packet and output how many have flow
		if (e->hasFlow) {
			double wxNew = e->u*DVS128_LOCAL_FLOW_TO_VENTRAL_FLOW;
			double wyNew = e->v*DVS128_LOCAL_FLOW_TO_VENTRAL_FLOW;
			state->wx += (wxNew-state->wx)/50;
			state->wy += (wyNew-state->wy)/50;
		}

		// Estimate time delay using last event
		if (caerEventPacketHeaderGetEventNumber((caerEventPacketHeader) flow) - i == 1) {
			delay = computeTimeDelay(state, e->timestamp);
		}
	}

	// TEST WRITE TO UART
	struct timespec currentTime;
	portable_clock_gettime_monotonic(&currentTime);
	if (!tested && state->uartConnectionReady){
		if (currentTime.tv_sec - state->timeInit.tv_sec >= 5) {
			uint8_t testData = 8;
			uart_tx(1,&testData);
			tested = true;
			fprintf(stdout,"Write\n");
		}
	}

	// Print average optic flow and time delay
	fprintf(stdout, "%c[2K", 27);
	fprintf(stdout, "\rwx: %1.3f. wy: %1.3f. timeDelay: %ld",
			state->wx, state->wy, delay/1000);
	fflush(stdout);
}

static void caerOpticFlowFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	OpticFlowFilterState state = moduleData->moduleState;

	state->refractoryPeriod = sshsNodeGetLong(moduleData->moduleNode, "refractoryPeriod");

	state->flowParams.dtMin = sshsNodeGetLong(moduleData->moduleNode, "flow_dtMin");
	state->flowParams.dtMax = sshsNodeGetLong(moduleData->moduleNode, "flow_dtMax");
	state->flowParams.dx 	= (uint16_t) sshsNodeGetInt(moduleData->moduleNode, "flow_dx");
	state->flowParams.thr1  = sshsNodeGetDouble(moduleData->moduleNode, "flow_thr1");
	state->flowParams.thr2  = sshsNodeGetDouble(moduleData->moduleNode, "flow_thr2");

	state->enableFlowRegularization = sshsNodeGetBool(moduleData->moduleNode, "filter_enable");
	state->filterParams.dtMax = sshsNodeGetLong(moduleData->moduleNode, "filter_dtMax");
	state->filterParams.dx = (uint16_t) sshsNodeGetInt(moduleData->moduleNode, "filter_dx");
	state->filterParams.maxSpeedFactor = sshsNodeGetDouble(moduleData->moduleNode, "filter_dMag");
	state->filterParams.maxAngle = sshsNodeGetDouble(moduleData->moduleNode, "filter_dAngle");

	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");
}

static void caerOpticFlowFilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	OpticFlowFilterState state = moduleData->moduleState;

	// Close UART connection
	if (state->uartConnectionReady) {
		uart_close();
	}

	// Ensure map is freed.
	flowEventBufferFree(state->buffer);
}

static bool allocateBuffer(OpticFlowFilterState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(sourceID));
	if (sourceInfoNode == NULL) {
		// This should never happen, but we handle it gracefully.
		caerLog(CAER_LOG_ERROR, __func__, "Failed to get source info to allocate flow event buffer.");
		return (false);
	}

	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	state->buffer = flowEventBufferInit((size_t) sizeX, (size_t) sizeY, FLOW_BUFFER_SIZE);
	if (state->buffer == NULL) {
		return (false);
	}

	// TODO: size the map differently if subSampleBy is set!
	return (true);
}

static int64_t computeTimeDelay(OpticFlowFilterState state, int64_t timeEvent) {
	struct timespec currentTime;
	portable_clock_gettime_monotonic(&currentTime);
	if (!state->timeSet || timeEvent < state->timeInitEvent) {
		state->timeInit = currentTime;
		state->timeInitEvent = timeEvent;
		state->timeSet = true;
		return (0);
	}
	else {
		int64_t timeS = currentTime.tv_sec - state->timeInit.tv_sec;
		int64_t timeUs = (currentTime.tv_nsec - state->timeInit.tv_nsec)/1000;
		int64_t timeSystem = timeS*1000000 + timeUs;
		int64_t delay = timeSystem - (timeEvent - state->timeInitEvent);
		// Make sure time delay >= 0 in case of incorrect initialization
		if (delay < 0) {
			state->timeInitEvent -= delay;
			delay = 0;
		}
		return (delay);
	}
}
