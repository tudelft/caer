/*
 * uartOutput.c
 *
 *  Created on: Jul 5, 2016
 *      Author: bas
 */

#include "uartOutput.h"
#define UART_SUBSYSTEM "UART"

static inline FlowEventPacket getPacketFromTransferBuffer(RingBuffer buffer);
static inline bool sendFlowEventPacketUart(FlowEventPacket packet);
static int uartHandlerThread(void *stateArg);

bool initUartOutput(uartState state, char* port, size_t bufferSize) {

	// Initialize UART communication
	int uartErrno = 0;//uart_open(port);
	if (uartErrno) {
		caerLog(CAER_LOG_ALERT, UART_SUBSYSTEM,"Failed to identify serial communication, errno=%i.",uartErrno);
		return (false);
	}
	// Start output handler thread
	state->thread = 0;
	if (thrd_create(&state->thread, &uartHandlerThread, state) != thrd_success) {
		caerLog(CAER_LOG_ALERT, UART_SUBSYSTEM,"Failed to start output handler thread");
		return (false);
	}
	state->buffer = ringBufferInit(bufferSize);
	if (state->buffer == NULL) {
		caerLog(CAER_LOG_ERROR, UART_SUBSYSTEM, "Failed to allocate transfer ring-buffer.");
		return (false);
	}
	atomic_store(&state->running, true);
	return (true);
}

void addPacketToTransferBuffer(uartState state, FlowEventPacket packet) {
	caerEventPacketHeader header = &packet->packetHeader;

	// There was nothing in this mainloop run!
	if (packet == NULL) {
		return;
	}
	// If no valid events are present, there is nothing to add
	if (caerEventPacketHeaderGetEventValid(header) == 0) {
		return;
	}

	FlowEventPacket copy = caerCopyEventPacketOnlyValidEvents(packet);

	if (copy == NULL) {
			caerLog(CAER_LOG_ERROR, UART_SUBSYSTEM,"Failed to copy event packet.");
		return;
	}
	if (!ringBufferPut(state->buffer, copy)) {
		free(copy);
		caerLog(CAER_LOG_ALERT, UART_SUBSYSTEM,"Failed to add event packet to ring buffer.");
		return;
	}
	printf("Packet added, size %4i\n",copy->packetHeader.eventNumber);
}

void closeUartOutput(uartState state) {
	state->running = false;
	//uart_close();

	if ((errno = thrd_join(state->thread, NULL)) != thrd_success) {
		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, UART_SUBSYSTEM,
				"Failed to join output handling thread. Error: %d.", errno);
	}
	ringBufferFree(state->buffer);
}

static inline FlowEventPacket getPacketFromTransferBuffer(RingBuffer buffer) {
	FlowEventPacket packet = ringBufferGet(buffer);
	repeat: if (packet != NULL) {
		// Are there others? Only render last one, to avoid getting backed up!
		FlowEventPacket packet2 = ringBufferGet(buffer);

		if (packet2 != NULL) {
			free(packet);
			packet = packet2;
			goto repeat;
		}
	}
	return (packet);
}

static inline bool sendFlowEventPacketUart(FlowEventPacket packet) {
	printf("Packet sent, size %3i\n",packet->packetHeader.eventNumber);
	return (true);
}

static int uartHandlerThread(void *stateArg) {
	if (stateArg == NULL) {
		return (thrd_error);
	}
	uartState state = stateArg;
	thrd_set_name(UART_SUBSYSTEM);
	struct timespec sleepTime = { .tv_sec = 0, .tv_nsec = 500000 };

	// Wait until the buffer is initialized
	while (!atomic_load_explicit(&state->running, memory_order_relaxed)) {
		thrd_sleep(&sleepTime, NULL);
	}

	// Main thread loop
	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {
		FlowEventPacket packet = getPacketFromTransferBuffer(state->buffer);
		if (packet == NULL) { // no data: sleep for a while
			thrd_sleep(&sleepTime, NULL);
		}
		else {
			if (!sendFlowEventPacketUart(packet)) {
				caerLog(CAER_LOG_ALERT, UART_SUBSYSTEM, "A flow packet was not sent.");
			}
			free(packet);
		}
	}

	// If shutdown: empty buffer before closing thread
	FlowEventPacket packet;
	while ((packet = getPacketFromTransferBuffer(state->buffer)) != NULL) {
		if (!sendFlowEventPacketUart(packet)) {
			caerLog(CAER_LOG_ALERT, UART_SUBSYSTEM, "A flow packet was not sent.");
		}
		free(packet);
	}

	return (thrd_success);
}
