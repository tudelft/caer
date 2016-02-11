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
#include "dvs128Calibration.h"
#include "modules/misc/inout_common.h"

#include <sys/statfs.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <limits.h>
#include <byteswap.h>

#define RING_BUFFER_SIZE 32768
#define DVS128_LOCAL_FLOW_TO_VENTRAL_FLOW 1/115.0f

outputMode outMode = OF_OUT_BOTH;

const char* UART_PORT = "/dev/ttySAC2"; // based on Odroid XU4 ports
unsigned int BAUD = B921600;

const char* RAW_OUTPUT_FILE_NAME = "rawEventLog";
const char* TIMING_OUTPUT_FILE_NAME = "timingLog";
const char* FLOW_OUTPUT_FILE_NAME = "flowEventLog";
int64_t RAW_EVENT_BYTES = 8; // approximate raw event storage size in bytes
int64_t EVENT_STORAGE_MARGIN = 100000000; // 100MB margin for storing events
int64_t maxNumberOfRawEvents;

struct OpticFlowFilter_state {
	flowAdaptiveState flowState;
	int8_t subSampleBy;
	float wx, wy, D;
	struct timespec timeInit;
	int64_t timeInitEvent;
	bool timeSet;
	flowOutputState outputState;
	FILE* rawOutputFile;
	FILE* timingOutputFile;

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
static bool openTimingFile(OpticFlowFilterState state, caerModuleData moduleData, const char* fileBaseName);
static bool openAEDatFile(OpticFlowFilterState state, caerModuleData moduleData, const char* fileBaseName);
static void writeAEDatFile(OpticFlowFilterState state, caerModuleData moduleData, flowEvent e);

static struct caer_module_functions caerOpticFlowFilterFunctions = { .moduleInit =
	&caerOpticFlowFilterInit, .moduleRun = &caerOpticFlowFilterRun, .moduleConfig =
	&caerOpticFlowFilterConfig, .moduleExit = &caerOpticFlowFilterExit };

void caerOpticFlowFilter(uint16_t moduleID, caerPolarityEventPacket polarity, flowEventPacket flow) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "OpticFlow", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerOpticFlowFilterFunctions, moduleData, sizeof(struct OpticFlowFilter_state),
			2, polarity, flow);
}

static bool caerOpticFlowFilterInit(caerModuleData moduleData) {
	sshsNodePutLongIfAbsent(moduleData->moduleNode,  "refractoryPeriod", 100000);
	sshsNodePutLongIfAbsent(moduleData->moduleNode,  "flow_dtMax",		2000000);
	sshsNodePutByteIfAbsent(moduleData->moduleNode,  "flow_dx",   		2);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "flow_vMax", 		1000.0f);
	sshsNodePutIntIfAbsent(moduleData->moduleNode,   "flow_rejectIterations", 	2);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "flow_maxNRMSE",0.3f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "flow_dtStopFactor",3.0f);
	sshsNodePutIntIfAbsent(moduleData->moduleNode,   "flow_nMin",8);

	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "adaptive_enable",false);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "adaptive_rateSP", 2500.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "adaptive_tau", 0.01f);

	sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);

	OpticFlowFilterState state = (OpticFlowFilterState)moduleData->moduleState;
	state->flowState = malloc(sizeof(struct flow_adaptive_state));
	state->flowState->refractoryPeriod = sshsNodeGetLong(moduleData->moduleNode,
			"refractoryPeriod");

	state->flowState->dtMax = sshsNodeGetLong(moduleData->moduleNode, "flow_dtMax");
	state->flowState->vMax = sshsNodeGetFloat(moduleData->moduleNode, "flow_vMax");
	state->flowState->dx = (uint8_t) sshsNodeGetByte(moduleData->moduleNode, "flow_dx");
	state->flowState->nReject = (uint32_t) sshsNodeGetInt(moduleData->moduleNode,
			"flow_rejectIterations");
	state->flowState->maxNRMSE  = sshsNodeGetFloat(moduleData->moduleNode,
			"flow_maxNRMSE");
	state->flowState->dtStopFactor  = sshsNodeGetFloat(moduleData->moduleNode,
			"flow_dtStopFactor");
	state->flowState->nMin = (uint32_t) sshsNodeGetInt(moduleData->moduleNode,
			"flow_nMin");

	state->flowState->limitEventRate = sshsNodeGetBool(moduleData->moduleNode, "adaptive_enable");

	state->flowState->rateSetpoint = sshsNodeGetFloat(moduleData->moduleNode,
			"adaptive_rateSP");
	state->flowState->rateTimeConstant = sshsNodeGetFloat(moduleData->moduleNode,
			"adaptive_tau");
	state->flowState->lastEventT = 0;
	state->flowState->flowRate = 0;

	flowAdaptiveInitSearchKernels(state->flowState);

	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

	state->wx = 0;
	state->wy = 0;
	state->D = 0;
	state->timeSet = false;

	state->lastFlowTimestamp = 0;

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// get path to head folder
	char* path = getLogPath(moduleData->moduleSubSystemString);

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
		// build absolute path
		char file_path[PATH_MAX];
		sprintf(file_path, "%s/%s", path, FLOW_OUTPUT_FILE_NAME);

		char comments[1024];
		sprintf(comments,"Parameters, refractoryPeriod, dtMax, vMax, dx, nReject, maxNRMSE, dtStopFactor, nMin, limitEventRate, rateSetpoint, rateTimeConstant, subSampleBy\n#, %lld, %lld, %f, %u, %u, %f, %f, %u, %u, %f, %f, %u\n",
		state->flowState->refractoryPeriod, state->flowState->dtMax, state->flowState->vMax, state->flowState->dx,
		state->flowState->nReject, state->flowState->maxNRMSE, state->flowState->dtStopFactor, state->flowState->nMin,
		state->flowState->limitEventRate, state->flowState->rateSetpoint, state->flowState->rateTimeConstant, state->subSampleBy);

		if (!initFileOutput(state->outputState, file_path, RING_BUFFER_SIZE, comments)) {
			caerLog(CAER_LOG_INFO,moduleData->moduleSubSystemString,
					"File logging not available.");
		}
		
		// open timging file
		openTimingFile(state, moduleData, path);
		openAEDatFile(state, moduleData, path);
	}
	free(path);

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

	if (flow == NULL) {
		return;
	}

	//flow = flowEventPacketInitFromPolarity(polarity);

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
		flowEvent e = flowEventPacketGetEvent(flow,i);

		// Skip invalid events.
		if (!flowEventIsValid(e)) {
			continue;
		}

		// Input logging
		if (state->outputState->mode == OF_OUT_FILE
				|| state->outputState->mode == OF_OUT_BOTH) {
			writeAEDatFile(state, moduleData, e);
		}

		uint16_t x = flowEventGetX(e);
		uint16_t y = flowEventGetY(e);
		bool p = flowEventGetPolarity(e);

		// Compute optic flow
		if (p) {
        	flowAdaptiveComputeFlow(e,state->lastTSMapOn,state->flowState);
		}
		else {
			flowAdaptiveComputeFlow(e,state->lastTSMapOff,state->flowState);
		}

		int64_t flowDt = e->timestamp - state->lastFlowTimestamp;

		if (e->hasFlow) {
			// For now, estimate average ventral flows/divergence for debugging purposes
			float wxNew = e->u / dvs128Calibration.focalLengthX;
			float wyNew = e->v / dvs128Calibration.focalLengthY;
			float dx = x - dvs128Calibration.principalPointX;
			float dy = y - dvs128Calibration.principalPointY;
			float DNew = 2*(dx*e->u + dy*e->v)/(dx*dx + dy*dy);
			float tFactor = ((float) flowDt) / 1E6f
					/ state->flowState->rateTimeConstant;
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
			if ((state->outputState->mode == OF_OUT_FILE
					|| state->outputState->mode == OF_OUT_BOTH)
					&& state->timingOutputFile != NULL) {
				fprintf(state->timingOutputFile, "%lld, %lld, %f, %f, %f, %f\n",
						e->timestamp, delay, state->flowState->flowRate,
						state->wx, state->wy, state->D);
			}
		}
	}
}

static void caerOpticFlowFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	OpticFlowFilterState state = moduleData->moduleState;

	state->flowState->refractoryPeriod = sshsNodeGetLong(moduleData->moduleNode,
			"refractoryPeriod");

	state->flowState->dtMax = sshsNodeGetLong(moduleData->moduleNode, "flow_dtMax");
	state->flowState->vMax = sshsNodeGetFloat(moduleData->moduleNode, "flow_vMax");
	state->flowState->dx = (uint8_t) sshsNodeGetByte(moduleData->moduleNode, "flow_dx");
	state->flowState->nReject = (uint32_t) sshsNodeGetInt(moduleData->moduleNode,
			"flow_rejectIterations");
	state->flowState->maxNRMSE  = sshsNodeGetFloat(moduleData->moduleNode,
			"flow_maxNRMSE");
	state->flowState->dtStopFactor  = sshsNodeGetFloat(moduleData->moduleNode,
			"flow_dtStopFactor");
	state->flowState->nMin = (uint32_t) sshsNodeGetInt(moduleData->moduleNode,
			"flow_nMin");

	state->flowState->limitEventRate = sshsNodeGetBool(moduleData->moduleNode, "adaptive_enable");
	state->flowState->rateSetpoint = sshsNodeGetFloat(moduleData->moduleNode,
			"adaptive_rateSP");
	state->flowState->rateTimeConstant = sshsNodeGetFloat(moduleData->moduleNode,
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

static bool openTimingFile(OpticFlowFilterState state, caerModuleData moduleData,
		const char* fileBaseName) {
	// Get time info
	time_t rawTime;
	time (&rawTime);
	const struct tm * timeInfo = localtime(&rawTime);
	char fileName[128];
	char fileTimestamp[64];
	strftime(fileTimestamp, sizeof(fileTimestamp),"%Y_%m_%d_%H_%M_%S", timeInfo);
	sprintf(fileName, "%s/%s_%s.csv", fileBaseName, TIMING_OUTPUT_FILE_NAME, fileTimestamp);

	// Check if filename exists
	struct stat st;
	int n = 0;
	while (stat(fileName,&st) == 0) {
		caerLog(CAER_LOG_WARNING, moduleData->moduleSubSystemString,
				"Filename %s is already used.", fileName);
		n++;
		sprintf(fileName, "%s/%s_%s_%d.csv", fileBaseName,
				TIMING_OUTPUT_FILE_NAME, fileTimestamp, n);
	}

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

static bool openAEDatFile(OpticFlowFilterState state, caerModuleData moduleData,
		const char* fileBaseName) {
	// In this (git) branch we also log the raw event input in an AEDAT1 file
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
	if (bytesFree <= EVENT_STORAGE_MARGIN && false)  {
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
	sprintf(fileName, "%s/%s_%s.aedat", fileBaseName, RAW_OUTPUT_FILE_NAME, fileTimestamp);

	// Check if filename exists
	struct stat st;
	int n = 0;
	while (stat(fileName,&st) == 0) {
		caerLog(CAER_LOG_WARNING, moduleData->moduleSubSystemString,
				"Filename %s is already used.", fileName);
		n++;
		sprintf(fileName, "%s/%s_%s_%d.aedat", fileBaseName,
				RAW_OUTPUT_FILE_NAME, fileTimestamp, n);
	}
	// Open file
	state->rawOutputFile = fopen(fileName,"w");
	if (state->rawOutputFile == NULL) {
		caerLog(CAER_LOG_ALERT, moduleData->moduleSubSystemString,
				"Failed to open file for raw event logging");
		return (false);
	}

	// Write header (based on output_common.c)
	fprintf(state->rawOutputFile,"#!AER-DAT2.0\n");
  fprintf(state->rawOutputFile,"# This is a raw AE data file - do not edit");
  fprintf(state->rawOutputFile,"# Data format is int16 address, int32 timestamp (4 bytes total), repeated for each event");
  fprintf(state->rawOutputFile,"# Timestamps tick is 1 us");
  size_t currentTimeStringLength = 45;
	char currentTimeString[currentTimeStringLength]; // + 1 for terminating NUL byte.
	strftime(currentTimeString, currentTimeStringLength, "# created %a %b %d %T %Z %G\n", timeInfo);
  char *sourceString = sshsNodeGetString(caerMainloopGetSourceInfo(10),
			"sourceString");
	fprintf(state->rawOutputFile,"%s",sourceString);
	free(sourceString);
  fprintf(state->rawOutputFile,"# AEChip: ch.unizh.ini.jaer.chip.retina.DVS128");

	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString,
			"Writing a maximum of %lld raw events to %s",
			maxNumberOfRawEvents, fileName);

	return (true);
}

static void writeAEDatFile(OpticFlowFilterState state, caerModuleData moduleData,
		flowEvent e) {
	static int64_t nEvents = 0;
	static uint32_t t, data, p, x, y;
	if (state->rawOutputFile != NULL) {
		if (nEvents < maxNumberOfRawEvents || true) {
			x = (uint32_t)((DVS128_N_PIXELS_Y - flowEventGetX(e) - 1) & 0x7f);
			y = (uint32_t)((DVS128_N_PIXELS_Y - flowEventGetY(e) - 1) & 0x7f);
			p = (uint32_t)(flowEventGetPolarity(e)  & 0x1);
			data = p | (x << 1) | (y << 8);	// arrange data in corret order for AEDat v2.0
			data = __bswap_32(data);	// write in big endian
			fwrite(&data, sizeof(data), 1, state->rawOutputFile);
			t = (uint32_t)(e->timestamp);
			t = __bswap_32(t);
			fwrite(&t, sizeof(t), 1, state->rawOutputFile);
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
