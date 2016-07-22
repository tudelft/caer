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
#include "flowOutput.h"
#include "termios.h"

#include <sys/statfs.h>

#define FLOW_BUFFER_SIZE 3
#define RING_BUFFER_SIZE 1024
#define DVS128_LOCAL_FLOW_TO_VENTRAL_FLOW 1/115.0

outputMode outMode = OF_OUT_FILE;

char* UART_PORT = (char*) "/dev/ttySAC2"; // based on Odroid XU4 ports
unsigned int BAUD = B921600;

char* RAW_OUTPUT_FILE_NAME = "logs/rawEventLog";
char* FLOW_OUTPUT_FILE_NAME = "logs/flowEventLog.csv";
int64_t RAW_EVENT_BYTES = 20; // approximate raw event storage size in bytes
int64_t EVENT_STORAGE_MARGIN = 100000000; // 100MB margin for storing events
bool LOG_AFTER_FILTERING = false;
int64_t MAX_N_RAW_EVENTS;

struct OpticFlowFilter_state {
	FlowEventBuffer buffer;
	FlowBenosman2014Params flowParams;
	FlowRegularizationFilterParams filterParams;
	bool enableFlowRegularization;
	int64_t refractoryPeriod;
	int8_t subSampleBy;
	double wx, wy;
	double flowRate;
	struct timespec timeInit;
	int64_t timeInitEvent;
	bool timeSet;
	flowOutputState outputState;
	FILE* rawOutputFile;
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

	state->outputState = malloc(sizeof(struct flow_output_state));
	atomic_store(&state->outputState->running, false);
	state->outputState->mode = outMode;
	if (outMode == OF_OUT_UART || outMode == OF_OUT_BOTH) {
		// Init UART communication
		if (!initUartOutput(state->outputState, UART_PORT, BAUD, RING_BUFFER_SIZE)) {
			caerLog(CAER_LOG_INFO,moduleData->moduleSubSystemString,
					"UART communication not available.");
		}
	}
	if (outMode == OF_OUT_FILE || outMode == OF_OUT_BOTH) {
		if (!initFileOutput(state->outputState, FLOW_OUTPUT_FILE_NAME, RING_BUFFER_SIZE)) {
			caerLog(CAER_LOG_INFO,moduleData->moduleSubSystemString,
					"UART communication not available.");
		}
	}

	// In this (git) branch we also log the raw event input in a separate file
	// Since storage space on the Odroid is limited, we check how much is available
	// and limit the log file size to this.
	struct statfs stat;
	if (statfs(".", &stat) != 0) {
		// error happens, just quits here
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
				"Raw event logging init failed at finding storage space.");
	}
	else {
		int64_t bytesFree = (int64_t) stat.f_bsize * (int64_t) stat.f_bavail;
		if (bytesFree <= EVENT_STORAGE_MARGIN) {
			caerLog(CAER_LOG_WARNING, moduleData->moduleSubSystemString,
					"Only %ld bytes available - raw logging disabled for safety", bytesFree);
		}
		else {
			MAX_N_RAW_EVENTS = (bytesFree - EVENT_STORAGE_MARGIN) / RAW_EVENT_BYTES;
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString,
					"%ld bytes available for logging", bytesFree);
			// Get timestamp info
			time_t rawTime;
			time (&rawTime);
			const struct tm * timeInfo = localtime(&rawTime);
			char fileName[128];
			char fileTimestamp[64];
			strftime(fileTimestamp, sizeof(fileTimestamp),"%Y_%m_%d_%T", timeInfo);
			sprintf(fileName, "%s_%s.csv", RAW_OUTPUT_FILE_NAME, fileTimestamp);
			// Open file
			state->rawOutputFile = fopen(fileName,"w");
			if (state->rawOutputFile == NULL) {
				caerLog(CAER_LOG_ALERT, moduleData->moduleSubSystemString,
						"Failed to open file for raw event logging");
				return (false);
			}
			// Write header comments
			fprintf(state->rawOutputFile,"#cAER raw event data\n");
			fprintf(state->rawOutputFile,"#Date created: %s\n",asctime(timeInfo));
			fprintf(state->rawOutputFile,"#x,y,t,p\n");
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString,
					"Writing a maximum of %ld raw events to %s",
					MAX_N_RAW_EVENTS, fileName);
		}
	}

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
	int32_t flowCount = 0;

	// Iterate over events and filter out ones that are not supported by other
	// events within a certain region in the specified timeframe.
	for (int32_t i = 0; i < caerEventPacketHeaderGetEventNumber((caerEventPacketHeader) flow); i++) {
		FlowEvent e = flowEventPacketGetEvent(flow,i);
		// Input logging
		uint16_t x = caerPolarityEventGetX((caerPolarityEvent) e);
		uint16_t y = caerPolarityEventGetY((caerPolarityEvent) e);
		if (!LOG_AFTER_FILTERING) {
			bool p = caerPolarityEventGetPolarity((caerPolarityEvent) e);
			if (state->rawOutputFile != NULL) {
				static int64_t nEvents = 0;
				if (nEvents < MAX_N_RAW_EVENTS) {
					fprintf(state->rawOutputFile, "%i,%i,%ld,%i\n",
							x,y,e->timestamp,p);
					nEvents++;
				}
			}
		}
		if (!caerPolarityEventIsValid((caerPolarityEvent) e)) { continue; } // Skip invalid events.

		// Refractory period
		FlowEvent eB = flowEventBufferRead(state->buffer,x,y,0);
		if (e->timestamp - eB->timestamp < state->refractoryPeriod) {
			flowEventBufferAdd(e, state->buffer); // preserve event but do not compute flow
			caerPolarityEventInvalidate((caerPolarityEvent) e,
					(caerPolarityEventPacket) flow);
			continue;
		}
		if (LOG_AFTER_FILTERING) {
			bool p = caerPolarityEventGetPolarity((caerPolarityEvent) e);
			if (state->rawOutputFile != NULL) {
				static int64_t nEvents = 0;
				if (nEvents < MAX_N_RAW_EVENTS) {
					fprintf(state->rawOutputFile, "%i,%i,%ld,%i\n",
							x,y,e->timestamp,p);
					nEvents++;
				}
			}
		}

		// Compute optic flow using events in buffer
		flowBenosman2014(e,state->buffer,state->flowParams);

		// Add event to buffer
		flowEventBufferAdd(e,state->buffer);

		// Apply flow regularization filter
		if (state->enableFlowRegularization) {
			flowRegularizationFilter(e,state->buffer,state->filterParams);
		}

		// For now, estimate average ventral flows for debugging purposes
		if (e->hasFlow) {
			double wxNew = e->u*DVS128_LOCAL_FLOW_TO_VENTRAL_FLOW;
			double wyNew = e->v*DVS128_LOCAL_FLOW_TO_VENTRAL_FLOW;
			state->wx += (wxNew-state->wx)/2;
			state->wy += (wyNew-state->wy)/2;
			flowCount++;
		}

		// Estimate time delay and flow event rate using last event
		if (caerEventPacketHeaderGetEventNumber((caerEventPacketHeader) flow) - i == 1) {
			delay = computeTimeDelay(state, e->timestamp);
			double packetTimeDiff = (double) (e->timestamp - flowEventPacketGetEvent(flow,0)->timestamp);
			double flowRate = flowCount / (packetTimeDiff+0.00001);
			state->flowRate += (flowRate-state->flowRate)/500;
		}
	}

	// Add event packet to ring buffer for transmission through UART/ to file
	// Transmission is performed in a separate thread
	if (atomic_load_explicit(&state->outputState->running, memory_order_relaxed)) {
		addPacketToTransferBuffer(state->outputState, flow, flowCount);
	}

	// Print average optic flow, time delay, and flow rate
//	fprintf(stdout, "%c[2K", 27);
//	fprintf(stdout, "\rwx: %1.3f. wy: %1.3f. delay: %ld ms. rate: %3.3fk",
//			state->wx, state->wy, delay/1000, state->flowRate*1e3);
//	fflush(stdout);
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

	// Close UART connection/file if necessary
	if (atomic_load_explicit(&state->outputState->running, memory_order_relaxed)) {
		if (state->outputState->mode == OF_OUT_UART
				|| state->outputState->mode == OF_OUT_BOTH) {
			closeUartOutput(state->outputState);
		}
		if (state->outputState->mode == OF_OUT_FILE
				|| state->outputState->mode == OF_OUT_BOTH) {
			closeFileOutput(state->outputState);
		}
	}
	free(state->outputState);
	if (state->rawOutputFile != NULL) {
		fclose(state->rawOutputFile);
	}

	// Ensure buffer is freed.
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
