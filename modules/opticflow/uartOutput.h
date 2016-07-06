/*
 * uartOutput.h
 *
 *  Created on: Jul 5, 2016
 *      Author: bas
 */

#ifndef MODULES_OPTICFLOW_UARTOUTPUT_H_
#define MODULES_OPTICFLOW_UARTOUTPUT_H_

#include "uart.h"
#include "flowEvent.h"
#include "ext/c11threads_posix.h"
#include "ext/ringbuffer/ringbuffer.h"

#include <stdatomic.h>

struct uart_state {
	atomic_bool running;
	RingBuffer buffer;
	thrd_t thread;
};

typedef struct uart_state *uartState;

bool initUartOutput(uartState state, char* port, size_t bufferSize);
void addPacketToTransferBuffer(uartState state, FlowEventPacket packet);
void closeUartOutput(uartState state);

#endif /* MODULES_OPTICFLOW_UARTOUTPUT_H_ */
