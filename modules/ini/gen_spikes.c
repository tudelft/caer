#include "base/module.h"
#ifdef HAVE_PTHREADS
#include "ext/c11threads_posix.h"
#endif

#include <math.h>
#include <stdatomic.h>
#include <sys/types.h>
#include <fcntl.h>
#include <time.h>

#include "main.h"
#include "dynapse_common.h"
#include "base/mainloop.h"
#include <libcaer/events/packetContainer.h>
#include <libcaer/events/spike.h>

#define STIM_POISSON 	1
#define STIM_REGULAR 	2
#define STIM_GAUSSIAN 	3
#define STIM_PATTERNA   4
#define STIM_PATTERNB   5
#define STIM_PATTERNC   6
#define STIM_PATTERNA_SINGLE   7
#define STIM_PATTERNB_SINGLE   8
#define STIM_PATTERNC_SINGLE   9
#define STIM_PATTERND_SINGLE   10

bool caerGenSpikeInit(caerModuleData moduleData);
void caerGenSpikeExit(caerModuleData moduleData);
int spikeGenThread(void *spikeGenState);
void spiketrainReg(void *spikeGenState);
void spiketrainPat(void *spikeGenState,
		uint32_t spikePattern[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE]);
void spiketrainPatSingle(void *spikeGenState, uint32_t sourceAddress);
void SetCam(void *spikeGenState);
void SetCamSingle(void *spikeGenState);
void ClearCam(void *spikeGenState);
void ClearAllCam(void *spikeGenState);
void WriteCam(void *spikeGenState, uint32_t preNeuronAddr,
		uint32_t postNeuronAddr, uint32_t camId, int16_t synapseType);
void ResetBiases(void *spikeGenState);
void setBiasBits(void *spikeGenState, uint32_t chipId, uint32_t coreId,
		const char *biasName_t, uint8_t coarseValue, uint16_t fineValue,
		const char *lowHigh, const char *npBias);

struct timespec tstart = { 0, 0 }, tend = { 0, 0 };
static int CamSeted = 0;
static int CamSetedSingle = 0;
static int CamCleared = 0;
static int CamAllCleared = 0;
static int BiasesLoaded = 0;
static int pattern_number = 4; //3 or 4

bool caerGenSpikeInit(caerModuleData moduleData) {

	caerInputDynapseState state = moduleData->moduleState;

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(moduleData->moduleNode,
			chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));

	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

	sshsNodePutBoolIfAbsent(spikeNode, "doStim", false); //false

	sshsNodePutIntIfAbsent(spikeNode, "stim_type", U8T(STIM_REGULAR)); //STIM_REGULAR
	atomic_store(&state->genSpikeState.stim_type,
			sshsNodeGetInt(spikeNode, "stim_type"));

	sshsNodePutIntIfAbsent(spikeNode, "stim_avr", 3);
	atomic_store(&state->genSpikeState.stim_avr,
			sshsNodeGetInt(spikeNode, "stim_avr"));

	sshsNodePutIntIfAbsent(spikeNode, "stim_std", 1);
	atomic_store(&state->genSpikeState.stim_std,
			sshsNodeGetInt(spikeNode, "stim_std"));

	sshsNodePutIntIfAbsent(spikeNode, "stim_duration", 10); //10
	atomic_store(&state->genSpikeState.stim_duration,
			sshsNodeGetInt(spikeNode, "stim_duration"));

	sshsNodePutBoolIfAbsent(spikeNode, "repeat", false); //false
	atomic_store(&state->genSpikeState.repeat,
			sshsNodeGetBool(spikeNode, "repeat"));

	sshsNodePutBoolIfAbsent(spikeNode, "teaching", true);
	atomic_store(&state->genSpikeState.teaching,
			sshsNodeGetBool(spikeNode, "teaching"));

	sshsNodePutBoolIfAbsent(spikeNode, "sendTeachingStimuli", true);
	atomic_store(&state->genSpikeState.sendTeachingStimuli,
			sshsNodeGetBool(spikeNode, "sendTeachingStimuli"));

	sshsNodePutBoolIfAbsent(spikeNode, "sendInhibitoryStimuli", false);
	atomic_store(&state->genSpikeState.sendInhibitoryStimuli,
			sshsNodeGetBool(spikeNode, "sendInhibitoryStimuli"));

	sshsNodePutBoolIfAbsent(spikeNode, "setCam", false); //1 //false
	atomic_store(&state->genSpikeState.setCam,
			sshsNodeGetBool(spikeNode, "setCam"));

	sshsNodePutBoolIfAbsent(spikeNode, "setCamSingle", false); //1 //false
	atomic_store(&state->genSpikeState.setCamSingle,
			sshsNodeGetBool(spikeNode, "setCamSingle"));

	sshsNodePutBoolIfAbsent(spikeNode, "clearCam", false); //1 //false
	atomic_store(&state->genSpikeState.clearCam,
			sshsNodeGetBool(spikeNode, "clearCam"));

	sshsNodePutBoolIfAbsent(spikeNode, "clearAllCam", false); //1 //false
	atomic_store(&state->genSpikeState.clearAllCam,
			sshsNodeGetBool(spikeNode, "clearAllCam"));

	sshsNodePutBoolIfAbsent(spikeNode, "doStimPrimitiveBias", true); //false
	atomic_store(&state->genSpikeState.doStimPrimitiveBias,
			sshsNodeGetBool(spikeNode, "doStimPrimitiveBias"));

	sshsNodePutBoolIfAbsent(spikeNode, "doStimPrimitiveCam", true); //false
	atomic_store(&state->genSpikeState.doStimPrimitiveCam,
			sshsNodeGetBool(spikeNode, "doStimPrimitiveCam"));

	sshsNodePutBoolIfAbsent(spikeNode, "loadDefaultBiases", false); //1 //false
	atomic_store(&state->genSpikeState.loadDefaultBiases,
			sshsNodeGetBool(spikeNode, "loadDefaultBiases"));

	atomic_store(&state->genSpikeState.started, false); //false
	atomic_store(&state->genSpikeState.done, true);

	// Start separate stimulation thread.
	atomic_store(&state->genSpikeState.running, true);

	if (thrd_create(&state->genSpikeState.spikeGenThread, &spikeGenThread,
			state) != thrd_success) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
				"SpikeGen: Failed to start thread.");
		return (NULL);
	}

	/*address*/
	sshsNodePutBoolIfAbsent(spikeNode, "sx", false);
	atomic_store(&state->genSpikeState.sx, sshsNodeGetBool(spikeNode, "sx"));

	sshsNodePutBoolIfAbsent(spikeNode, "sy", false);
	atomic_store(&state->genSpikeState.sy, sshsNodeGetBool(spikeNode, "sy"));

	sshsNodePutIntIfAbsent(spikeNode, "core_d", 0);
	atomic_store(&state->genSpikeState.core_d,
			sshsNodeGetInt(spikeNode, "core_d"));

	sshsNodePutIntIfAbsent(spikeNode, "core_s", 0);
	atomic_store(&state->genSpikeState.core_s,
			sshsNodeGetInt(spikeNode, "core_s"));

	sshsNodePutIntIfAbsent(spikeNode, "address", 1);
	atomic_store(&state->genSpikeState.address,
			sshsNodeGetInt(spikeNode, "address"));

	sshsNodePutIntIfAbsent(spikeNode, "dx", 0);
	atomic_store(&state->genSpikeState.dx, sshsNodeGetInt(spikeNode, "dx"));

	sshsNodePutIntIfAbsent(spikeNode, "dy", 0);
	atomic_store(&state->genSpikeState.dy, sshsNodeGetInt(spikeNode, "dy"));

	sshsNodePutIntIfAbsent(spikeNode, "chip_id", DYNAPSE_CONFIG_DYNAPSE_U0); //4
	atomic_store(&state->genSpikeState.chip_id,
			sshsNodeGetInt(spikeNode, "chip_id"));

	return (true);
}

void caerGenSpikeExit(caerModuleData moduleData) {
	caerInputDynapseState state = moduleData->moduleState;

	//make sure that doStim is off
	size_t biasNodesLength = 0;
	sshsNode *biasNodesU0 = sshsNodeGetChildren(state->eventSourceConfigNode,
			&biasNodesLength);
	// find the spikeGen node
	if (biasNodesU0 != NULL) {
		for (size_t i = 0; i < biasNodesLength; i++) {
			if (caerStrEquals("DYNAPSEFX2", sshsNodeGetName(biasNodesU0[i]))) {
				sshsNode *biasNodesU1 = sshsNodeGetChildren(biasNodesU0[i],
						&biasNodesLength);
				if (biasNodesU1 != NULL) {
					for (size_t i = 0; i < biasNodesLength; i++) {
						if (caerStrEquals("spikeGen",
								sshsNodeGetName(biasNodesU1[i]))) {
							sshsNodePutBool(biasNodesU1[i], "doStim",
									false);
							caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "doStim has been set back to false.");
						}
					}
				}
				free(biasNodesU1);
			}
		}
		free(biasNodesU0);
	}

	// Shut down stimulation thread and wait on it to finish.
	atomic_store(&state->genSpikeState.running, false);

	if ((errno = thrd_join(state->genSpikeState.spikeGenThread, NULL))
			!= thrd_success) {
		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
				"SpikeGen: Failed to join rendering thread. Error: %d.", errno);
	}

}

int spikeGenThread(void *spikeGenState) {
	if (spikeGenState == NULL) {
		return (thrd_error);
	}

	caerInputDynapseState state = spikeGenState;

	thrd_set_name("SpikeGenThread");

	while (atomic_load_explicit(&state->genSpikeState.running, // the loop
			memory_order_relaxed)) {

		if (state->genSpikeState.setCam == true && CamSeted == 0) {
			SetCam(state);
			CamSeted = 1;
		} else if (state->genSpikeState.setCam == false && CamSeted == 1) {
			CamSeted = 0;
		}
		if (state->genSpikeState.setCamSingle == true && CamSetedSingle == 0) {
			SetCamSingle(state);
			CamSetedSingle = 1;
		} else if (state->genSpikeState.setCamSingle == false
				&& CamSetedSingle == 1) {
			CamSetedSingle = 0;
		}
		if (state->genSpikeState.clearCam == true && CamCleared == 0) {
			ClearCam(state);
			CamCleared = 1;
		} else if (state->genSpikeState.clearCam == false && CamCleared == 1) {
			CamCleared = 0;
		}
		if (state->genSpikeState.clearAllCam == true && CamAllCleared == 0) {
			ClearAllCam(state);
			CamAllCleared = 1;
		} else if (state->genSpikeState.clearAllCam == false
				&& CamAllCleared == 1) {
			CamAllCleared = 0;
		}
		if (state->genSpikeState.loadDefaultBiases == true
				&& BiasesLoaded == 0) {
			ResetBiases(state);
			BiasesLoaded = 1;
		} else if (state->genSpikeState.loadDefaultBiases == false
				&& BiasesLoaded == 1) {
			BiasesLoaded = 0;
		}

		/* generate spikes*/

		if (state->genSpikeState.stim_type == STIM_REGULAR) {
			spiketrainReg(state);
		} else if (state->genSpikeState.stim_type == STIM_POISSON) {

		} else if (state->genSpikeState.stim_type == STIM_GAUSSIAN) {

		} else if (state->genSpikeState.stim_type == STIM_PATTERNA) {
			// generate pattern A
			uint32_t spikePatternA[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE];
			int64_t rowId, colId;
			int cx, cy, r;
			cx = 16;
			cy = 16;
			r = 14;
			for (rowId = 0; rowId < DYNAPSE_CONFIG_XCHIPSIZE; rowId++)
				for (colId = 0; colId < DYNAPSE_CONFIG_YCHIPSIZE; colId++)
					spikePatternA[rowId][colId] = 0;
			for (rowId = cx - r; rowId <= cx + r; rowId++)
				for (colId = cy - r; colId <= cy + r; colId++)
					if (((cx - rowId) * (cx - rowId)
							+ (cy - colId) * (cy - colId) <= r * r + sqrt(r))
							&& ((cx - rowId) * (cx - rowId)
									+ (cy - colId) * (cy - colId) >= r * r - r))
						spikePatternA[rowId][colId] = 1;
			spiketrainPat(state, spikePatternA);
		} else if (state->genSpikeState.stim_type == STIM_PATTERNB) {
			//generate pattern B
			uint32_t spikePatternB[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE];
			int64_t rowId, colId;
			int64_t num = DYNAPSE_CONFIG_CAMNUM;
			for (rowId = -num; rowId < num; rowId++) {
				for (colId = -num; colId < num; colId++) {
					if (abs((int) rowId) + abs((int) colId) == num) // Change this condition >= <=
						spikePatternB[rowId + DYNAPSE_CONFIG_CAMNUM][colId
								+ DYNAPSE_CONFIG_CAMNUM] = 1;
					else
						spikePatternB[rowId + DYNAPSE_CONFIG_CAMNUM][colId
								+ DYNAPSE_CONFIG_CAMNUM] = 0;
				}
			}
			spiketrainPat(state, spikePatternB);
		} else if (state->genSpikeState.stim_type == STIM_PATTERNC) {
			//generate pattern C
			uint32_t spikePatternC[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE];
			int64_t rowId, colId;
			int64_t num = DYNAPSE_CONFIG_CAMNUM;
			for (rowId = -num; rowId < num; rowId++) {
				for (colId = -num; colId < num; colId++) {
					if (abs((int) rowId) == abs((int) colId)) // Change this condition
						spikePatternC[rowId + DYNAPSE_CONFIG_CAMNUM][colId
								+ DYNAPSE_CONFIG_CAMNUM] = 1;
					else
						spikePatternC[rowId + DYNAPSE_CONFIG_CAMNUM][colId
								+ DYNAPSE_CONFIG_CAMNUM] = 0;
				}
			}
			spiketrainPat(state, spikePatternC);
		} else if (state->genSpikeState.stim_type == STIM_PATTERNA_SINGLE) {
			// generate pattern A
			uint32_t sourceAddress = 1;
			spiketrainPatSingle(state, sourceAddress);
		} else if (state->genSpikeState.stim_type == STIM_PATTERNB_SINGLE) {
			//generate pattern B
			uint32_t sourceAddress = 2;
			spiketrainPatSingle(state, sourceAddress);
		} else if (state->genSpikeState.stim_type == STIM_PATTERNC_SINGLE) {
			//generate pattern C
			uint32_t sourceAddress = 3;
			spiketrainPatSingle(state, sourceAddress);
		} else if (state->genSpikeState.stim_type == STIM_PATTERND_SINGLE) {
			//generate pattern D
			uint32_t sourceAddress = 4;
			spiketrainPatSingle(state, sourceAddress);
		}

	}

	return (thrd_success);
}

void spiketrainReg(void *spikeGenState) {

	if (spikeGenState == NULL) {
		return;
	}

	caerInputDynapseState state = spikeGenState;

	struct timespec tim;
	tim.tv_sec = 0;

	if (atomic_load(&state->genSpikeState.stim_avr) > 0) {
		tim.tv_nsec = 1000000000L / atomic_load(&state->genSpikeState.stim_avr);
	} else {
		tim.tv_nsec = 1000000000L;
	}

	uint32_t value = atomic_load(&state->genSpikeState.core_d) | 0 << 16
			| 0 << 17 | 1 << 13 |
			atomic_load(&state->genSpikeState.core_s) << 18 |
			atomic_load(&state->genSpikeState.address) << 20 |
			atomic_load(&state->genSpikeState.dx) << 4 |
			atomic_load(&state->genSpikeState.sx) << 6 |
			atomic_load(&state->genSpikeState.dy) << 7 |
			atomic_load(&state->genSpikeState.sy) << 9;

	if (!atomic_load(&state->genSpikeState.started)) {
		LABELSTART: clock_gettime(CLOCK_MONOTONIC, &tstart);
	}

	clock_gettime(CLOCK_MONOTONIC, &tend);

	if (atomic_load(&state->genSpikeState.stim_duration)
			<= ((double) tend.tv_sec + 1.0e-9 * tend.tv_nsec)
					- ((double) tstart.tv_sec + 1.0e-9 * tstart.tv_nsec)) {
		if (atomic_load(&state->genSpikeState.started)) {
			caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation finished.\n");
		}
		atomic_store(&state->genSpikeState.done, true);
		atomic_store(&state->genSpikeState.started, false);
		if (atomic_load(&state->genSpikeState.repeat)) {
			caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation re-started.\n");
			atomic_store(&state->genSpikeState.started, true);
			atomic_store(&state->genSpikeState.done, false);
			goto LABELSTART;
		}
	}

	if (!atomic_load(&state->genSpikeState.done)) {
		nanosleep(&tim, NULL);
		// send spikes
		caerDeviceConfigSet((caerDeviceHandle) state->deviceState,
		DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
				atomic_load(&state->genSpikeState.chip_id));  //usb_handle
		/*send the spike*/
		caerDeviceConfigSet((caerDeviceHandle) state->deviceState,
		DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, value); //usb_handle
		//caerLog(CAER_LOG_NOTICE, "spikeGen", "sending spikes %d \n", value);
	}

}

void spiketrainPat(void *spikeGenState,
		uint32_t spikePattern[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE]) { //generate and send 32*32 input stimuli

	if (spikeGenState == NULL) {
		return;
	}

	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;

	struct timespec tim;
	tim.tv_sec = 0;

	if (atomic_load(&state->genSpikeState.stim_avr) > 0) {
		tim.tv_nsec = 1000000000L / atomic_load(&state->genSpikeState.stim_avr);
	} else {
		tim.tv_nsec = 1000000000L; //1000000000L;
	}

	//generate chip command for stimulating
	uint32_t value, valueSent;
	uint32_t value2DArray[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE];
	int64_t rowId, colId;
	for (rowId = 0; rowId < DYNAPSE_CONFIG_XCHIPSIZE; rowId++)
		for (colId = 0; colId < DYNAPSE_CONFIG_YCHIPSIZE; colId++) {
			if (spikePattern[rowId][colId] == 1)
				value = 0xf | 0 << 16 | 0 << 17 | 1 << 13
						| (((rowId / DYNAPSE_CONFIG_NEUROW) << 1)
								| (colId / DYNAPSE_CONFIG_NEUCOL)) << 18
						| (((rowId % DYNAPSE_CONFIG_NEUROW) << 4)
								| (colId % DYNAPSE_CONFIG_NEUCOL)) << 20 |
						atomic_load(&state->genSpikeState.dx) << 4 |
						atomic_load(&state->genSpikeState.sx) << 6 |
						atomic_load(&state->genSpikeState.dy) << 7 |
						atomic_load(&state->genSpikeState.sy) << 9;
			else
				value = 0;
			value2DArray[rowId][colId] = value;
		}

	if (!atomic_load(&state->genSpikeState.started)) {
		LABELSTART: clock_gettime(CLOCK_MONOTONIC, &tstart);
	}

	clock_gettime(CLOCK_MONOTONIC, &tend);

	if (atomic_load(&state->genSpikeState.stim_duration)
			<= ((double) tend.tv_sec + 1.0e-9 * tend.tv_nsec)
					- ((double) tstart.tv_sec + 1.0e-9 * tstart.tv_nsec)) {
		if (atomic_load(&state->genSpikeState.started)) {
			caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation finished.\n");
		}
		atomic_store(&state->genSpikeState.done, true);
		atomic_store(&state->genSpikeState.started, false);
		if (atomic_load(&state->genSpikeState.repeat)) {
			caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation re-started.\n");
			atomic_store(&state->genSpikeState.started, true);
			atomic_store(&state->genSpikeState.done, false);
			goto LABELSTART;
		}
	}

	if (!atomic_load(&state->genSpikeState.done)) {
		nanosleep(&tim, NULL);
		// send spikes
		caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP,
		DYNAPSE_CONFIG_CHIP_ID, atomic_load(&state->genSpikeState.chip_id));
		//send the spike
		for (rowId = 0; rowId < DYNAPSE_CONFIG_XCHIPSIZE; rowId++)
			for (colId = 0; colId < DYNAPSE_CONFIG_YCHIPSIZE; colId++) {
				valueSent = value2DArray[rowId][colId];
				if (valueSent != 0 && ((valueSent >> 18) & 0x3ff) != 0) {
					caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP,
					DYNAPSE_CONFIG_CHIP_CONTENT, valueSent);
				}
			}
		//caerLog(CAER_LOG_NOTICE, "spikeGen", "sending spikes %d \n", value);
	}

}

void spiketrainPatSingle(void *spikeGenState, uint32_t sourceAddress) {
	if (spikeGenState == NULL) {
		return;
	}

	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;

	struct timespec tim;
	tim.tv_sec = 0;

	if (atomic_load(&state->genSpikeState.stim_avr) > 0) {
		tim.tv_nsec = 1000000000L / atomic_load(&state->genSpikeState.stim_avr);
	} else {
		tim.tv_nsec = 1000000000L;
	}

	//generate chip command for stimulating
	uint32_t valueSent, valueSentTeaching, valueSentTeachingControl,
			valueSentInhibitory, valueSentInhibitoryControl;
	uint32_t source_address;
	valueSent = 0xf | 0 << 16 | 0 << 17 | 1 << 13 | (sourceAddress & 0xff) << 20
			| ((sourceAddress & 0x300) >> 8) << 18 |
			atomic_load(&state->genSpikeState.dx) << 4 |
			atomic_load(&state->genSpikeState.sx) << 6 |
			atomic_load(&state->genSpikeState.dy) << 7 |
			atomic_load(&state->genSpikeState.sy) << 9;

	if (pattern_number == 3) {
		if ((sourceAddress & 0xff) == 1) {
			source_address = 0;
		} else if ((sourceAddress & 0xff) == 2) {
			source_address = 4;
		} else if ((sourceAddress & 0xff) == 3) {
			source_address = 8;
		}
	} else if (pattern_number == 4) {
		if ((sourceAddress & 0xff) == 1) {
			source_address = 0;
		} else if ((sourceAddress & 0xff) == 2) {
			source_address = 4;
		} else if ((sourceAddress & 0xff) == 3) {
			source_address = 8;
		} else if ((sourceAddress & 0xff) == 4) {
			source_address = 12;
		}
	}

	valueSentTeaching = 0x8 | 0 << 16 | 0 << 17 | 1 << 13 | source_address << 20
			| 0x3 << 18 |
			atomic_load(&state->genSpikeState.dx) << 4 |
			atomic_load(&state->genSpikeState.sx) << 6 |
			atomic_load(&state->genSpikeState.dy) << 7 |
			atomic_load(&state->genSpikeState.sy) << 9; //((sourceAddress & 0x300) >> 8) << 18

	valueSentTeachingControl = 0xc | 0 << 16 | 0 << 17 | 1 << 13
			| source_address << 20 | 0x3 << 18 |
			atomic_load(&state->genSpikeState.dx) << 4 |
			atomic_load(&state->genSpikeState.sx) << 6 | 1 << 7 | 1 << 9;

	valueSentInhibitory = 0x8 | 0 << 16 | 0 << 17 | 1 << 13 | 3 << 20
			| 0x3 << 18 |
			atomic_load(&state->genSpikeState.dx) << 4 |
			atomic_load(&state->genSpikeState.sx) << 6 |
			atomic_load(&state->genSpikeState.dy) << 7 |
			atomic_load(&state->genSpikeState.sy) << 9; //((sourceAddress & 0x300) >> 8) << 18

	valueSentInhibitoryControl = 0xc | 0 << 16 | 0 << 17 | 1 << 13 | 3 << 20
			| 0x3 << 18 |
			atomic_load(&state->genSpikeState.dx) << 4 |
			atomic_load(&state->genSpikeState.sx) << 6 | 1 << 7 | 1 << 9;

	if (!atomic_load(&state->genSpikeState.started)) {
		LABELSTART: clock_gettime(CLOCK_MONOTONIC, &tstart);
	}

	clock_gettime(CLOCK_MONOTONIC, &tend);

	if (atomic_load(&state->genSpikeState.stim_duration)
			<= ((double) tend.tv_sec + 1.0e-9 * tend.tv_nsec)
					- ((double) tstart.tv_sec + 1.0e-9 * tstart.tv_nsec)) {
		if (atomic_load(&state->genSpikeState.started)) {
			caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation finished.\n");
		}
		atomic_store(&state->genSpikeState.done, true);
		atomic_store(&state->genSpikeState.started, false);
		if (atomic_load(&state->genSpikeState.repeat)) {
			caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation re-started.\n");
			atomic_store(&state->genSpikeState.started, true);
			atomic_store(&state->genSpikeState.done, false);
			goto LABELSTART;
		}
	}

	if (!atomic_load(&state->genSpikeState.done)) {
		nanosleep(&tim, NULL);
		// send spikes
		if (atomic_load(&state->genSpikeState.doStimPrimitiveBias) == true
				&& atomic_load(&state->genSpikeState.doStimPrimitiveCam) == true) {
			caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP,
			DYNAPSE_CONFIG_CHIP_ID, atomic_load(&state->genSpikeState.chip_id));
			//send the spike
			caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP,
			DYNAPSE_CONFIG_CHIP_CONTENT, valueSent);
			if (atomic_load(&state->genSpikeState.teaching) == true
					&& atomic_load(&state->genSpikeState.sendTeachingStimuli)
							== true) {
				caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP,
				DYNAPSE_CONFIG_CHIP_ID,
				DYNAPSE_CONFIG_DYNAPSE_U2);
				caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP,
				DYNAPSE_CONFIG_CHIP_CONTENT, valueSentTeaching);
				caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP,
				DYNAPSE_CONFIG_CHIP_CONTENT, valueSentTeachingControl);
			}
			if (atomic_load(&state->genSpikeState.sendInhibitoryStimuli) == true) { //atomic_load(&state->genSpikeState.teaching) == true &&
				caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP,
				DYNAPSE_CONFIG_CHIP_ID,
				DYNAPSE_CONFIG_DYNAPSE_U2);
				caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP,
				DYNAPSE_CONFIG_CHIP_CONTENT, valueSentInhibitory);
				caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP,
				DYNAPSE_CONFIG_CHIP_CONTENT, valueSentInhibitoryControl);
			}
		}
		//caerLog(CAER_LOG_NOTICE, "spikeGen", "sending spikes %d \n", valueSent);
	}
}

void SetCam(void *spikeGenState) {
	if (spikeGenState == NULL) {
		return;
	}
	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
			atomic_load(&state->genSpikeState.chip_id)); //0
	uint32_t neuronId;
	caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "Started programming cam..");
	for (neuronId = 0;
			neuronId < DYNAPSE_CONFIG_XCHIPSIZE * DYNAPSE_CONFIG_YCHIPSIZE;
			neuronId++) {
		WriteCam(state, neuronId, neuronId, 0, 3);
	}
	caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "CAM programmed successfully.");

	// set back setCam to false
	size_t biasNodesLength = 0;
	sshsNode *biasNodesU0 = sshsNodeGetChildren(state->eventSourceConfigNode,
			&biasNodesLength);
	// find the spikeGen node
	if (biasNodesU0 != NULL) {
		for (size_t i = 0; i < biasNodesLength; i++) {
			if (caerStrEquals("DYNAPSEFX2", sshsNodeGetName(biasNodesU0[i]))) {
				sshsNode *biasNodesU1 = sshsNodeGetChildren(biasNodesU0[i],
						&biasNodesLength);
				if (biasNodesU1 != NULL) {
					for (size_t i = 0; i < biasNodesLength; i++) {
						if (caerStrEquals("spikeGen",
								sshsNodeGetName(biasNodesU1[i]))) {
							sshsNodePutBool(biasNodesU1[i], "setCam",
									false);
							caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "setCam has been set back to false.");
						}
					}
				}
				free(biasNodesU1);
			}
		}
		free(biasNodesU0);
	}

}

void SetCamSingle(void *spikeGenState) {
	if (spikeGenState == NULL) {
		return;
	}
	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
			atomic_load(&state->genSpikeState.chip_id)); //0

	int64_t rowId, colId;
	int64_t num = DYNAPSE_CONFIG_CAMNUM;
	// generate pattern A
	uint32_t spikePatternA[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE];
	int cx, cy, r;
	cx = 16;
	cy = 16;
	r = 14;
	for (rowId = 0; rowId < DYNAPSE_CONFIG_XCHIPSIZE; rowId++)
		for (colId = 0; colId < DYNAPSE_CONFIG_YCHIPSIZE; colId++)
			spikePatternA[rowId][colId] = 0;
	for (rowId = cx - r; rowId <= cx + r; rowId++)
		for (colId = cy - r; colId <= cy + r; colId++)
			if (((cx - rowId) * (cx - rowId) + (cy - colId) * (cy - colId)
					<= r * r + sqrt(r))
					&& ((cx - rowId) * (cx - rowId)
							+ (cy - colId) * (cy - colId) >= r * r - r))
				spikePatternA[rowId][colId] = 1;

	uint32_t spikePatternB[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE];
	for (rowId = -num; rowId < num; rowId++) {
		for (colId = -num; colId < num; colId++) {
			if (abs((int) rowId) + abs((int) colId) == num) // Change this condition >= <=
				spikePatternB[rowId + DYNAPSE_CONFIG_CAMNUM][colId
						+ DYNAPSE_CONFIG_CAMNUM] = 1;
			else
				spikePatternB[rowId + DYNAPSE_CONFIG_CAMNUM][colId
						+ DYNAPSE_CONFIG_CAMNUM] = 0;
		}
	}

	uint32_t spikePatternC[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE];
	for (rowId = -num; rowId < num; rowId++) {
		for (colId = -num; colId < num; colId++) {
			if (abs((int) rowId) == abs((int) colId)) // Change this condition
				spikePatternC[rowId + DYNAPSE_CONFIG_CAMNUM][colId
						+ DYNAPSE_CONFIG_CAMNUM] = 1;
			else
				spikePatternC[rowId + DYNAPSE_CONFIG_CAMNUM][colId
						+ DYNAPSE_CONFIG_CAMNUM] = 0;
		}
	}

	uint32_t neuronId;
	caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "Started programming cam..");
	for (rowId = 0; rowId < DYNAPSE_CONFIG_XCHIPSIZE; rowId++) {
		for (colId = 0; colId < DYNAPSE_CONFIG_YCHIPSIZE; colId++) {
			neuronId = ((rowId & 0X10) >> 4) << 9 | ((colId & 0X10) >> 4) << 8
					| (rowId & 0xf) << 4 | colId & 0xf;
			if (spikePatternA[rowId][colId] == 1)
				WriteCam(state, 1, neuronId, 0, 3);
			if (spikePatternB[rowId][colId] == 1)
				WriteCam(state, 2, neuronId, 1, 3);
			if (spikePatternC[rowId][colId] == 1)
				WriteCam(state, 3, neuronId, 2, 3);
		}
	}

	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
	DYNAPSE_CONFIG_DYNAPSE_U2); //4, the third chip
	neuronId = 3 << 8 | 0;
	WriteCam(state, 1, neuronId, 61, 3);
	WriteCam(state, 2, neuronId, 62, 1);
	WriteCam(state, 3, neuronId, 63, 1);
	neuronId = 3 << 8 | 1;
	WriteCam(state, 1, neuronId, 61, 1);
	WriteCam(state, 2, neuronId, 62, 3);
	WriteCam(state, 3, neuronId, 63, 1);
	neuronId = 3 << 8 | 2;
	WriteCam(state, 1, neuronId, 61, 1);
	WriteCam(state, 2, neuronId, 62, 1);
	WriteCam(state, 3, neuronId, 63, 3);

	caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "CAM programmed successfully.");

	// set back clearAllCam to false
	size_t biasNodesLength = 0;
	sshsNode *biasNodesU0 = sshsNodeGetChildren(state->eventSourceConfigNode,
			&biasNodesLength);
	// find the spikeGen node
	if (biasNodesU0 != NULL) {
		for (size_t i = 0; i < biasNodesLength; i++) {
			if (caerStrEquals("DYNAPSEFX2", sshsNodeGetName(biasNodesU0[i]))) {
				sshsNode *biasNodesU1 = sshsNodeGetChildren(biasNodesU0[i],
						&biasNodesLength);
				if (biasNodesU1 != NULL) {
					for (size_t i = 0; i < biasNodesLength; i++) {
						if (caerStrEquals("spikeGen",
								sshsNodeGetName(biasNodesU1[i]))) {
							sshsNodePutBool(biasNodesU1[i], "setCamSingle",
									false);
							caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "setCamSingle has been set back to false.");
						}
					}
				}
				free(biasNodesU1);
			}
		}
		free(biasNodesU0);
	}

}

void ClearCam(void *spikeGenState) {
	if (spikeGenState == NULL) {
		return;
	}
	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
			atomic_load(&state->genSpikeState.chip_id)); //0
	uint32_t neuronId;
	caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "Started clearing cam...");
	caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "please wait...");
	for (neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS; neuronId++) {
		WriteCam(state, 0, neuronId, 0, 0);
	}
	caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "Done, CAM cleared successfully.");
	atomic_store(&state->genSpikeState.clearCam, false);

	// set back clearCam to false
	size_t biasNodesLength = 0;
	sshsNode *biasNodesU0 = sshsNodeGetChildren(state->eventSourceConfigNode,
			&biasNodesLength);
	// find the spikeGen node
	if (biasNodesU0 != NULL) {
		for (size_t i = 0; i < biasNodesLength; i++) {
			if (caerStrEquals("DYNAPSEFX2", sshsNodeGetName(biasNodesU0[i]))) {
				sshsNode *biasNodesU1 = sshsNodeGetChildren(biasNodesU0[i],
						&biasNodesLength);
				if (biasNodesU1 != NULL) {
					for (size_t i = 0; i < biasNodesLength; i++) {
						if (caerStrEquals("spikeGen",
								sshsNodeGetName(biasNodesU1[i]))) {
							sshsNodePutBool(biasNodesU1[i], "clearCam",
									false);
							caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "clearCam has been set back to false.");
						}
					}
				}
				free(biasNodesU1);
			}
		}
		free(biasNodesU0);
	}

}

void ClearAllCam(void *spikeGenState) {
	if (spikeGenState == NULL) {
		return;
	}
	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
			atomic_load(&state->genSpikeState.chip_id));
	uint32_t neuronId, camId;
	caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "Started clearing cam..");
	for (neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS; neuronId++) {
		for (camId = 0; camId < DYNAPSE_X4BOARD_NEUX; camId++) {
			WriteCam(state, 0, neuronId, camId, 0);
		}
	}
	caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "CAM cleared successfully.");
	atomic_store(&state->genSpikeState.clearAllCam, false);

	// set back clearAllCam to false
	size_t biasNodesLength = 0;
	sshsNode *biasNodesU0 = sshsNodeGetChildren(state->eventSourceConfigNode,
			&biasNodesLength);
	// find the spikeGen node
	if (biasNodesU0 != NULL) {
		for (size_t i = 0; i < biasNodesLength; i++) {
			if (caerStrEquals("DYNAPSEFX2", sshsNodeGetName(biasNodesU0[i]))) {
				sshsNode *biasNodesU1 = sshsNodeGetChildren(biasNodesU0[i],
						&biasNodesLength);
				if (biasNodesU1 != NULL) {
					for (size_t i = 0; i < biasNodesLength; i++) {
						if (caerStrEquals("spikeGen",
								sshsNodeGetName(biasNodesU1[i]))) {
							sshsNodePutBool(biasNodesU1[i], "clearAllCam",
									false);
							caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "clearAllCam has been set back to false.");
						}
					}
				}
				free(biasNodesU1);
			}
		}
		free(biasNodesU0);
	}

}

void WriteCam(void *spikeGenState, uint32_t preNeuronAddr,
		uint32_t postNeuronAddr, uint32_t camId, int16_t synapseType) {
	if (spikeGenState == NULL) {
		return;
	}
	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;

	uint32_t bits;
	uint32_t ei = (synapseType & 0x2) >> 1;
	uint32_t fs = synapseType & 0x1;
	uint32_t address = preNeuronAddr & 0xff;
	uint32_t source_core = (preNeuronAddr & 0x300) >> 8;
	uint32_t coreId = (postNeuronAddr & 0x300) >> 8;
	uint32_t neuron_row = (postNeuronAddr & 0xf0) >> 4;
	uint32_t synapse_row = camId;
	uint32_t row = neuron_row << 6 | synapse_row;
	uint32_t column = postNeuronAddr & 0xf;
	bits = ei << 29 | fs << 28 | address << 20 | source_core << 18 | 1 << 17
			| coreId << 15 | row << 5 | column;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP,
	DYNAPSE_CONFIG_CHIP_CONTENT, bits);
}

void ResetBiases(void *spikeGenState) {

	caerLog(CAER_LOG_NOTICE, "loadDefaultBiases", "started...");
	caerInputDynapseState state = spikeGenState;
	struct caer_dynapse_info dynapse_info = caerDynapseInfoGet(
			state->deviceState);

	if (spikeGenState == NULL) {
		return;
	}

	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	uint32_t chipId_t, chipId, coreId;

	for (chipId_t = 0; chipId_t < 1; chipId_t++) {

		if (chipId_t == 0)
			chipId = DYNAPSE_CONFIG_DYNAPSE_U0;
		else if (chipId_t == 1)
			chipId = DYNAPSE_CONFIG_DYNAPSE_U2;
		else if (chipId_t == 2)
			chipId = DYNAPSE_CONFIG_DYNAPSE_U1;
		else if (chipId_t == 3)
			chipId = DYNAPSE_CONFIG_DYNAPSE_U3;

		caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP,
		DYNAPSE_CONFIG_CHIP_ID, chipId);

		for (coreId = 0; coreId < 4; coreId++) {
			/*if(chipId_t == 0 && coreId == 0){
			 setBiasBits(state, chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
			 setBiasBits(state, chipId, coreId, "IF_AHTHR_N", 7, 1, "HighBias", "NBias");
			 setBiasBits(state, chipId, coreId, "IF_AHW_P", 7, 1, "HighBias", "PBias");
			 setBiasBits(state, chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
			 setBiasBits(state, chipId, coreId, "IF_CASC_N", 7, 1, "HighBias", "NBias");
			 setBiasBits(state, chipId, coreId, "IF_DC_P", 5, 2, "HighBias", "PBias");
			 setBiasBits(state, chipId, coreId, "IF_NMDA_N", 7, 1, "HighBias", "NBias");
			 setBiasBits(state, chipId, coreId, "IF_RFR_N", 2, 180, "HighBias", "NBias");
			 setBiasBits(state, chipId, coreId, "IF_TAU1_N", 6, 8, "LowBias", "NBias");
			 setBiasBits(state, chipId, coreId, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
			 setBiasBits(state, chipId, coreId, "IF_THR_N", 3, 50, "HighBias", "NBias");
			 }else if(chipId_t == 0 && coreId == 1){*/
			setBiasBits(state, chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias",
					"NBias");
			setBiasBits(state, chipId, coreId, "IF_AHTHR_N", 7, 1, "HighBias",
					"NBias");
			setBiasBits(state, chipId, coreId, "IF_AHW_P", 7, 1, "HighBias",
					"PBias");
			setBiasBits(state, chipId, coreId, "IF_BUF_P", 3, 80, "HighBias",
					"PBias");
			setBiasBits(state, chipId, coreId, "IF_CASC_N", 7, 1, "HighBias",
					"NBias");
			setBiasBits(state, chipId, coreId, "IF_DC_P", 5, 2, "HighBias",
					"PBias");
			setBiasBits(state, chipId, coreId, "IF_NMDA_N", 7, 1, "HighBias",
					"NBias");
			setBiasBits(state, chipId, coreId, "IF_RFR_N", 2, 180, "HighBias",
					"NBias");
			setBiasBits(state, chipId, coreId, "IF_TAU1_N", 4, 255, "LowBias",
					"NBias");
			setBiasBits(state, chipId, coreId, "IF_TAU2_N", 6, 15, "HighBias",
					"NBias");
			setBiasBits(state, chipId, coreId, "IF_THR_N", 2, 180, "HighBias",
					"NBias");
			setBiasBits(state, chipId, coreId, "NPDPIE_TAU_F_P", 6, 150,
					"HighBias", "PBias");
			setBiasBits(state, chipId, coreId, "NPDPIE_TAU_S_P", 7, 40,
					"HighBias", "NBias");
			setBiasBits(state, chipId, coreId, "NPDPIE_THR_F_P", 0, 220,
					"HighBias", "PBias");
			setBiasBits(state, chipId, coreId, "NPDPIE_THR_S_P", 7, 0,
					"HighBias", "PBias");
			setBiasBits(state, chipId, coreId, "NPDPII_TAU_F_P", 6, 150,
					"HighBias", "NBias");
			setBiasBits(state, chipId, coreId, "NPDPII_TAU_S_P", 7, 40,
					"HighBias", "NBias");
			setBiasBits(state, chipId, coreId, "NPDPII_THR_F_P", 0, 220,
					"HighBias", "PBias");
			setBiasBits(state, chipId, coreId, "NPDPII_THR_S_P", 7, 0,
					"HighBias", "PBias");
			setBiasBits(state, chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 250,
					"HighBias", "NBias");
			setBiasBits(state, chipId, coreId, "PS_WEIGHT_EXC_S_N", 1, 250,
					"HighBias", "NBias");
			setBiasBits(state, chipId, coreId, "PS_WEIGHT_INH_F_N", 0, 250,
					"HighBias", "NBias");
			setBiasBits(state, chipId, coreId, "PS_WEIGHT_INH_S_N", 1, 250,
					"HighBias", "NBias");
			setBiasBits(state, chipId, coreId, "PULSE_PWLK_P", 3, 50,
					"HighBias", "PBias");
			setBiasBits(state, chipId, coreId, "R2R_P", 4, 85, "HighBias",
					"PBias");
		}
	}

	// set back clearAllCam to false
	size_t biasNodesLength = 0;
	sshsNode *biasNodesU0 = sshsNodeGetChildren(state->eventSourceConfigNode,
			&biasNodesLength);
	// find the spikeGen node
	if (biasNodesU0 != NULL) {
		for (size_t i = 0; i < biasNodesLength; i++) {
			if (caerStrEquals("DYNAPSEFX2", sshsNodeGetName(biasNodesU0[i]))) {
				sshsNode *biasNodesU1 = sshsNodeGetChildren(biasNodesU0[i],
						&biasNodesLength);
				if (biasNodesU1 != NULL) {
					for (size_t i = 0; i < biasNodesLength; i++) {
						if (caerStrEquals("spikeGen",
								sshsNodeGetName(biasNodesU1[i]))) {
							sshsNodePutBool(biasNodesU1[i], "loadDefaultBiases",
									false);
							caerLog(CAER_LOG_NOTICE, "\nSpikeGen", "loadDefaultBiases has been set back to false.");
						}
					}
				}
				free(biasNodesU1);
			}
		}
		free(biasNodesU0);
	}

}

void setBiasBits(void *spikeGenState, uint32_t chipId, uint32_t coreId,
		const char *biasName_t, uint8_t coarseValue, uint16_t fineValue,
		const char *lowHigh, const char *npBias) {

	if (spikeGenState == NULL) {
		return;
	}

	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	struct caer_dynapse_info dynapse_info = caerDynapseInfoGet(
			state->deviceState);

	size_t biasNameLength = strlen(biasName_t);
	char biasName[biasNameLength + 3];

	biasName[0] = 'C';
	if (coreId == 0)
		biasName[1] = '0';
	else if (coreId == 1)
		biasName[1] = '1';
	else if (coreId == 2)
		biasName[1] = '2';
	else if (coreId == 3)
		biasName[1] = '3';
	biasName[2] = '_';

	uint32_t i;
	for (i = 0; i < biasNameLength + 3; i++) {
		biasName[3 + i] = biasName_t[i];
	}

	uint32_t bits = generatesBitsCoarseFineBiasSetting(
			state->eventSourceConfigNode, &dynapse_info, biasName, coarseValue,
			fineValue, lowHigh, "Normal", npBias, true, chipId);

	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP,
	DYNAPSE_CONFIG_CHIP_CONTENT, bits);
}
