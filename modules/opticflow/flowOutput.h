/*
 * uartOutput.h
 *
 *  Created on: Jul 5, 2016
 *      Author: bas
 */

#ifndef MODULES_OPTICFLOW_FLOWOUTPUT_H_
#define MODULES_OPTICFLOW_FLOWOUTPUT_H_

#include "uart.h"
#include "flowEvent.h"
#include "ext/c11threads_posix.h"
#include "ext/ringbuffer/ringbuffer.h"
#include <stdatomic.h>

typedef enum {
	OF_OUT_NONE,
	OF_OUT_FILE,
	OF_OUT_UART,
	OF_OUT_BOTH
} outputMode;

struct flow_output_state {
	atomic_bool running;
	outputMode mode;
	RingBuffer buffer;
	thrd_t thread;
	FILE* file;
	int32_t fileLineNumber;
};

typedef struct flow_output_state *flowOutputState;

bool initUartOutput(flowOutputState state, char* port, unsigned int baud, size_t bufferSize);
void closeUartOutput(flowOutputState state);

bool initFileOutput(flowOutputState state, char* file, size_t bufferSize);
void closeFileOutput(flowOutputState state);

void addPacketToTransferBuffer(flowOutputState state, FlowEventPacket packet, int32_t flowNumber);

#endif /* MODULES_OPTICFLOW_FLOWOUTPUT_H_ */
