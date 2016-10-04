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

/**
 * Output mode definitions.
 */
typedef enum {
	OF_OUT_NONE,//!< No flow output
	OF_OUT_FILE,//!< Output to files
	OF_OUT_UART,//!< Output to UART connection
	OF_OUT_BOTH //!< Output to both files and UART
} outputMode;

/**
 * Output thread state variables.
 */
struct flow_output_state {
	atomic_bool running;
	outputMode mode;
	RingBuffer buffer;
	thrd_t thread;
	FILE* file;
	int32_t fileLineNumber;
};

typedef struct flow_output_state *flowOutputState;

/**
 * Initialization of UART communication link.
 * @param state Flow output state.
 * @param port UART port of this machine.
 * @param baud Baud rate specification.
 * @param bufferSize Transfer ring buffer size.
 * @return true if successful, false if error
 */
bool initUartOutput(flowOutputState state, const char* port, unsigned int baud, size_t bufferSize);

/**
 * Termination of UART communication link, free up state memory.
 * @param state Flow output state.
 */
void closeUartOutput(flowOutputState state);

/**
 * File logging initialization and header setup.
 * @param state Flow output state.
 * @param file Basename of file (is appended with date/time of creation)
 * @param bufferSize Transfer ring buffer size.
 * @return
 */
bool initFileOutput(flowOutputState state, const char* file, size_t bufferSize);

/**
 * Close file logging and free up state memory.
 * @param state
 */
void closeFileOutput(flowOutputState state);

/**
 * Add flow event to buffer, included in the flow output state
 * @param state output state, which contains the event buffer
 * @param e the new event to be added
 */
void addFlowEventToTransferBuffer(flowOutputState state, flowEvent e);

#endif /* MODULES_OPTICFLOW_FLOWOUTPUT_H_ */
