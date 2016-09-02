/*
 * opticflow.c
 *
 *  Created on: Jun 13, 2016
 *      Author: bas
 */

#include "opticflow.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "ext/portable_time.h"
#include "flowEvent.h"
#include "flowAdaptive.h"
#include "flowOutput.h"
#include "termios.h"

#include <sys/statfs.h>
#include <sys/stat.h>
#include <sys/types.h>

#define FLOW_BUFFER_SIZE 3
#define RING_BUFFER_SIZE 16384
#define DVS128_LOCAL_FLOW_TO_VENTRAL_FLOW 1/115.0f

outputMode outMode = OF_OUT_FILE;

char* UART_PORT = (char*) "/dev/ttySAC2"; // based on Odroid XU4 ports
unsigned int BAUD = B921600;

char* RAW_OUTPUT_FILE_NAME = "logs/rawEventLog";
char* TIMING_OUTPUT_FILE_NAME = "logs/timingLog";
char* FLOW_OUTPUT_FILE_NAME = "logs/flowEventLog";
int64_t RAW_EVENT_BYTES = 8; // approximate raw event storage size in bytes
int64_t EVENT_STORAGE_MARGIN = 100000000; // 100MB margin for storing events
int64_t maxNumberOfRawEvents;

struct OpticFlowFilter_state {
	flowAdaptiveState flowState;
	int8_t subSampleBy;
	float wx, wy, D;
	float flowRate;
	struct timespec timeInit;
	int64_t timeInitEvent;
	bool timeSet;
	flowOutputState outputState;
	FILE* rawOutputFile;
	FILE* timingOutputFile;

	bool enableAdaptiveFilter;
	int64_t lastFlowTimestamp;
	simple2DBufferLong lastTSMapOn;
	simple2DBufferLong lastTSMapOff;
};

typedef struct OpticFlowFilter_state *OpticFlowFilterState;

static bool caerOpticFlowFilterInit(caerModuleData moduleData);
static void caerOpticFlowFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerOpticFlowFilterConfig(caerModuleData moduleData);
static void caerOpticFlowFilterExit(caerModuleData moduleData);
static bool allocateBuffers(OpticFlowFilterState state, int16_t sourceID);
static int64_t computeTimeDelay(OpticFlowFilterState state, int64_t timeEvent);
static bool openAEDatFile(OpticFlowFilterState state, caerModuleData moduleData, char* fileBaseName);
static void writeAEDatFile(OpticFlowFilterState state, caerModuleData moduleData, flowEvent e);

static struct caer_module_functions caerOpticFlowFilterFunctions = { .moduleInit =
	&caerOpticFlowFilterInit, .moduleRun = &caerOpticFlowFilterRun, .moduleConfig =
	&caerOpticFlowFilterConfig, .moduleExit = &caerOpticFlowFilterExit };

void caerOpticFlowFilter(uint16_t moduleID, caerPolarityEventPacket polarity, flowEventPacket flow) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "OpticFlow");
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerOpticFlowFilterFunctions, moduleData, sizeof(struct OpticFlowFilter_state),
			2, polarity, flow);
}

static bool caerOpticFlowFilterInit(caerModuleData moduleData) {
	sshsNodePutLongIfAbsent(moduleData->moduleNode, "refractoryPeriod", 10000);

	sshsNodePutLongIfAbsent(moduleData->moduleNode,  "flow_dtMax",				200000);
	sshsNodePutByteIfAbsent(moduleData->moduleNode,  "flow_dx",   				3);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "flow_vMax", 				200.0f);
	sshsNodePutIntIfAbsent(moduleData->moduleNode,   "flow_rejectIterations", 	2);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "flow_rejectDistance",		5E3f);

	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "adaptive_enable",true);
	sshsNodePutIntIfAbsent(moduleData->moduleNode,  "adaptive_nMin", 5);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "adaptive_rateSP", 600.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "adaptive_gain", 2.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "adaptive_tau", 0.01f);

	sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);

	OpticFlowFilterState state = moduleData->moduleState;
	state->flowState = malloc(sizeof(struct flow_adaptive_state));
	state->flowState->refractoryPeriod = sshsNodeGetLong(moduleData->moduleNode,
			"refractoryPeriod");

	state->flowState->dtMax = sshsNodeGetLong(moduleData->moduleNode, "flow_dtMax");
	state->flowState->vMax = sshsNodeGetFloat(moduleData->moduleNode, "flow_vMax");
	state->flowState->dx = (uint8_t) sshsNodeGetByte(moduleData->moduleNode, "flow_dx");
	state->flowState->rejectIterations = (uint32_t) sshsNodeGetInt(moduleData->moduleNode,
			"flow_rejectIterations");
	state->flowState->rejectDistance  = sshsNodeGetFloat(moduleData->moduleNode,
			"flow_rejectDistance");

	state->enableAdaptiveFilter = sshsNodeGetBool(moduleData->moduleNode, "adaptive_enable");
	state->flowState->adaptiveNMin = (uint32_t) sshsNodeGetInt(moduleData->moduleNode,
			"adaptive_nMin");
	state->flowState->adaptiveRateSetpoint = sshsNodeGetFloat(moduleData->moduleNode,
			"adaptive_rateSP");
	state->flowState->adaptiveNGain = sshsNodeGetFloat(moduleData->moduleNode,"adaptive_gain");
	state->flowState->adaptiveTimeConstant = sshsNodeGetFloat(moduleData->moduleNode,
			"adaptive_tau");

	flowAdaptiveInitSearchKernels(state->flowState);

	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

	state->wx = 0;
	state->wy = 0;
	state->D = 0;
	state->flowRate = 0;
	state->timeSet = false;

	state->lastFlowTimestamp = 0;

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
					"File logging not available.");
		}
	}
	if (outMode == OF_OUT_FILE || outMode == OF_OUT_BOTH) {
		openAEDatFile(state, moduleData, RAW_OUTPUT_FILE_NAME);
	}

	return (true);
}

static void caerOpticFlowFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
	flowEventPacket flow = va_arg(args, flowEventPacket);

	// Only process packets with content.
	if (polarity == NULL) {
		return;
	}

#ifdef ENABLE_VISUALIZER
	if (flow == NULL) {
		return;
	}
#endif


	OpticFlowFilterState state = moduleData->moduleState;

	// If last timestamp maps are not allocated yet, do so now
	if (state->lastTSMapOn == NULL) {
		if (!allocateBuffers(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
					"Failed to allocate memory for timestamp maps.");
			return;
		}
	}

	int64_t delay = 0;
	int32_t flowCount = 0;

	// Iterate over all events.
	for (int32_t i = 0; i < caerEventPacketHeaderGetEventNumber(&polarity->packetHeader); i++) {
		// In all cases, we work with flow events in this module
#ifdef ENABLE_VISUALIZER
		// Flow event is readily available from flow packet
		flowEvent e = flowEventPacketGetEvent(flow,i);
#else
		// We need to separately identify the flow event
		caerPolarityEvent pol = caerPolarityEventPacketGetEvent(polarity,i);
		struct flow_event fEvent = flowEventInitFromPolarity(pol,polarity);
		flowEvent e = &fEvent;
#endif
		// Input logging
		if (state->outputState->mode == OF_OUT_FILE
				|| state->outputState->mode == OF_OUT_BOTH) {
			writeAEDatFile(state, moduleData, e);
		}
		// Skip invalid events.
		if (!flowEventIsValid(e)) {
			continue;
		}

		uint8_t x = flowEventGetX(e);
		uint8_t y = flowEventGetY(e);
		bool p = flowEventGetPolarity(e);

		// Compute optic flow
		if (p) {
        	flowAdaptiveComputeFlow(e,state->lastTSMapOn,state->flowState);
        	simple2DBufferLongSet(state->lastTSMapOn, x, y, e->timestamp);
		}
		else {
			flowAdaptiveComputeFlow(e,state->lastTSMapOff,state->flowState);
			simple2DBufferLongSet(state->lastTSMapOff, x, y, e->timestamp);
		}

		int64_t flowDt = e->timestamp - state->lastFlowTimestamp;
		flowAdaptiveUpdateSetpoint(state->flowState, flowDt);

		if (e->hasFlow) {
			// For now, estimate average ventral flows for debugging purposes
			float wxNew = e->u*DVS128_LOCAL_FLOW_TO_VENTRAL_FLOW;
			float wyNew = e->v*DVS128_LOCAL_FLOW_TO_VENTRAL_FLOW;
			float dx = x - 76.7f;
			float dy = y - 56.93f;
			float DNew = 2*(dx*e->u + dy*e->v)/(dx*dx + dy*dy);
			float tFactor = ((float) flowDt) / 1E6f
					/ state->flowState->adaptiveTimeConstant;
			if (tFactor > 0.1f) tFactor = 0.1f;
			state->wx += (wxNew-state->wx)*tFactor;
			state->wy += (wyNew-state->wy)*tFactor;
			state->D += (DNew-state->D)*tFactor;
			flowCount++;

			state->lastFlowTimestamp = e->timestamp;

			// Add event to ring buffer for transmission through UART/ to file
			// Transmission is performed in a separate thread
			if (atomic_load_explicit(&state->outputState->running,
					memory_order_relaxed)) {
				addFlowEventToTransferBuffer(state->outputState, e);
			}
		}

		// Estimate time delay and flow event rate using last event
		if (caerEventPacketHeaderGetEventNumber(&polarity->packetHeader) - i == 1) {
			delay = computeTimeDelay(state, e->timestamp);
			// Log timing info
			if (state->outputState->mode == OF_OUT_FILE
					|| state->outputState->mode == OF_OUT_BOTH) {
				fprintf(state->timingOutputFile, "%ld, %ld, %f, %f, %f, %f\n",
						e->timestamp, delay, state->flowState->flowRate,
						state->wx, state->wy, state->D);
			}
		}
	}

	// Print average optic flow, time delay, and flow rate
	fprintf(stdout, "%c[2K", 27);
	fprintf(stdout, "\rwx: %1.3f. wy: %1.3f. D: %1.3f. delay: %ld ms. rate: %3.3fk",
			state->wx, state->wy, state->D, delay/1000, state->flowState->flowRate/1e3f);
	fflush(stdout);
}

static void caerOpticFlowFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	OpticFlowFilterState state = moduleData->moduleState;

	state->flowState->refractoryPeriod = sshsNodeGetLong(moduleData->moduleNode,
			"refractoryPeriod");

	state->flowState->dtMax = sshsNodeGetLong(moduleData->moduleNode, "flow_dtMax");
	state->flowState->vMax = sshsNodeGetFloat(moduleData->moduleNode, "flow_vMax");
	state->flowState->dx = (uint8_t) sshsNodeGetByte(moduleData->moduleNode, "flow_dx");
	state->flowState->rejectIterations = (uint32_t) sshsNodeGetInt(moduleData->moduleNode,
			"flow_rejectIterations");
	state->flowState->rejectDistance  = sshsNodeGetFloat(moduleData->moduleNode,
			"flow_rejectDistance");

	state->enableAdaptiveFilter = sshsNodeGetBool(moduleData->moduleNode, "adaptive_enable");
	state->flowState->adaptiveNMin = (uint32_t) sshsNodeGetInt(moduleData->moduleNode,
			"adaptive_nMin");
	state->flowState->adaptiveRateSetpoint = sshsNodeGetFloat(moduleData->moduleNode,
			"adaptive_rateSP");
	state->flowState->adaptiveNGain = sshsNodeGetFloat(moduleData->moduleNode,"adaptive_gain");
	state->flowState->adaptiveTimeConstant = sshsNodeGetFloat(moduleData->moduleNode,
			"adaptive_tau");

	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

	// Re-initialize search kernels, note that this is necessary whenever the dx parameter
	// is modified.
	flowAdaptiveFreeSearchKernels(state->flowState);
	flowAdaptiveInitSearchKernels(state->flowState);
}

static void caerOpticFlowFilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData,
			&caerModuleConfigDefaultListener);

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
			// Close direct logging files
			if (state->rawOutputFile != NULL) {
				fclose(state->rawOutputFile);
			}
			if (state->timingOutputFile != NULL) {
				fclose(state->timingOutputFile);
			}
		}
	}

	// Free file/UART thread state memory
	free(state->outputState);

	// Free buffer memory
	flowAdaptiveFreeSearchKernels(state->flowState);
	simple2DBufferFreeLong(state->lastTSMapOn);
	simple2DBufferFreeLong(state->lastTSMapOff);

	// Free flow state variable
	free(state->flowState);
}

static bool allocateBuffers(OpticFlowFilterState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(sourceID));
	if (sourceInfoNode == NULL) {
		// This should never happen, but we handle it gracefully.
		caerLog(CAER_LOG_ERROR, __func__,
				"Failed to get source info to allocate flow event buffer.");
		return (false);
	}

	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	state->lastTSMapOn = simple2DBufferInitLong((size_t) sizeX, (size_t) sizeY);
	state->lastTSMapOff = simple2DBufferInitLong((size_t) sizeX, (size_t) sizeY);
	if (state->lastTSMapOn == NULL || state->lastTSMapOff == NULL) {
		return (false);
	}

	return (true);
}

static int64_t computeTimeDelay(OpticFlowFilterState state, int64_t timeEvent) {
	struct timespec currentTime;
	portable_clock_gettime_monotonic(&currentTime);
	// (re-)initialize
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

static bool openAEDatFile(OpticFlowFilterState state, caerModuleData moduleData,
		char* fileBaseName) {
	// In this (git) branch we also log the raw event input in an AEDAT3.1 file
	// Since storage space on the Odroid is limited, we check how much is available
	// and limit the log file size to this.
	struct statfs stfs;
	if (statfs(".", &stfs) != 0) {
		// error happens, just quits here
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
				"Raw event logging init failed at finding storage space.");
		return (false);
	}
	int64_t bytesFree = (int64_t) stfs.f_bsize * (int64_t) stfs.f_bavail;
	if (bytesFree <= EVENT_STORAGE_MARGIN) {
		caerLog(CAER_LOG_WARNING, moduleData->moduleSubSystemString,
				"Only %lld bytes available - raw logging disabled for safety", bytesFree);
		return (false);
	}
	maxNumberOfRawEvents = (bytesFree - EVENT_STORAGE_MARGIN) / RAW_EVENT_BYTES;
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString,
			"%lld bytes available for logging", bytesFree);

	// Get time info
	time_t rawTime;
	time (&rawTime);
	const struct tm * timeInfo = localtime(&rawTime);
	char fileName[128];
	char fileTimestamp[64];
	strftime(fileTimestamp, sizeof(fileTimestamp),"%Y_%m_%d_%H_%M_%S", timeInfo);
	sprintf(fileName, "%s_%s.aedat", fileBaseName, fileTimestamp);

	// Check if filename exists
	struct stat st;
	int n = 0;
	while (stat(fileName,&st) == 0) {
		caerLog(CAER_LOG_WARNING, moduleData->moduleSubSystemString,
				"Filename %s is already used.", fileName);
		n++;
		sprintf(fileName, "%s_%s_%d.aedat",
				fileBaseName, fileTimestamp, n);
	}
	// Open file
	state->rawOutputFile = fopen(fileName,"w");
	if (state->rawOutputFile == NULL) {
		caerLog(CAER_LOG_ALERT, moduleData->moduleSubSystemString,
				"Failed to open file for raw event logging");
		return (false);
	}

	// Write header (based on output_common.c)
	fprintf(state->rawOutputFile,"#!AER-DAT3.0\n");
	fprintf(state->rawOutputFile,"#Format: RAW\r\n");
	char *sourceString = sshsNodeGetString(caerMainloopGetSourceInfo(1),
			"sourceString");
	fprintf(state->rawOutputFile,sourceString);
	free(sourceString);

	size_t currentTimeStringLength = 45;
	char currentTimeString[currentTimeStringLength]; // + 1 for terminating NUL byte.
	strftime(currentTimeString, currentTimeStringLength,
			"#Start-Time: %Y-%m-%d %H:%M:%S (TZ%z)\n", timeInfo);

	fprintf(state->rawOutputFile, currentTimeString);
	fprintf(state->rawOutputFile, "#!END-HEADER\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString,
			"Writing a maximum of %lld raw events to %s",
			maxNumberOfRawEvents, fileName);

	// Now we also log time delay
	sprintf(fileName, "%s_%s.csv",
			TIMING_OUTPUT_FILE_NAME, fileTimestamp);
	if (n > 0)
		sprintf(fileName, "%s_%s_%d.csv",
				fileBaseName, fileTimestamp, n);
	state->timingOutputFile = fopen(fileName,"w");
	if (state->timingOutputFile == NULL) {
		caerLog(CAER_LOG_ALERT, moduleData->moduleSubSystemString,
				"Failed to open file for timing logging");
		return (false);
	}
	fprintf(state->timingOutputFile, "#Timing data for each event packet\n");
	fprintf(state->timingOutputFile,
			"#timestamp [us], delay [us], flowRate [1/s], wx [1/s], wy [1/s], D [1/s]\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString,
			"Writing timing info to %s",fileName);

	return (true);
}

static void writeAEDatFile(OpticFlowFilterState state, caerModuleData moduleData,
		flowEvent e) {
	if (state->rawOutputFile != NULL) {
		static int64_t nEvents = 0;
		if (nEvents < maxNumberOfRawEvents) {
			fwrite(&e->data, 1, sizeof(e->data), state->rawOutputFile);
			int32_t t = (int32_t) e->timestamp;
			fwrite(&t, 1, sizeof(t), state->rawOutputFile);
			nEvents++;
		}
		else {
			caerLog(CAER_LOG_ALERT, moduleData->moduleSubSystemString,
					"Raw log size limit reached - terminating logging");
			fclose(state->rawOutputFile);
			state->rawOutputFile = NULL;
		}
	}
}
