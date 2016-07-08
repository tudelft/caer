/*
 * uartOutput.c
 *
 *  Created on: Jul 5, 2016
 *      Author: bas
 */

#include "flowOutput.h"
#define SUBSYSTEM_UART "UART"
#define SUBSYSTEM_FILE "FILEOUT"

static inline caerEventPacketHeader getPacketFromTransferBuffer(RingBuffer buffer);
static inline bool sendFlowEventPacketUart(caerEventPacketHeader header);
static int outputHandlerThread(void *stateArg);
static inline bool writeFlowEventPacketFile(flowOutputState state,
		caerEventPacketHeader packet);

bool initUartOutput(flowOutputState state, char* port, unsigned int baud, size_t bufferSize) {
	// Initialize UART communication
	int uartErrno = uart_open(port, baud);
	if (uartErrno) {
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART,
				"Failed to identify serial communication, errno=%i.",uartErrno);
		return (false);
	}
	// Test message
	char* testMessage = "First message \n";
	if (uart_tx((int) strlen(testMessage), (unsigned char*) testMessage)) {
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART,
				"Test transmission unsuccessful - connection closed.");
		uart_close();
		return(false);
	}

	// Start output handler thread
	state->thread = 0;
	if (thrd_create(&state->thread, &outputHandlerThread, state) != thrd_success) {
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART,"Failed to start output handler thread");
		return (false);
	}
	state->buffer = ringBufferInit(bufferSize);
	if (state->buffer == NULL) {
		caerLog(CAER_LOG_ERROR, SUBSYSTEM_UART, "Failed to allocate transfer ring-buffer.");
		return (false);
	}
	atomic_store(&state->running, true);
	caerLog(CAER_LOG_NOTICE, SUBSYSTEM_UART, "Streaming flow events to port %s.",port);
	return (true);
}

bool initFileOutput(flowOutputState state, char* fileName, size_t bufferSize) {
	// Initialize file communication
	state->file = fopen(fileName,"w");
	if (state->file == NULL) {
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE,"Failed to open file");
		return (false);
	}
	// Write header

	if (fprintf(state->file,"#AER data with optic flow values\n") < 0) {
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE,"Failed to write header");
		fclose(state->file);
		return (false);
	}
	// Write creation date
	time_t rawTime;
	struct tm * timeInfo;
	time (&rawTime);
	timeInfo = localtime(&rawTime);
	fprintf(state->file,"#Date: %s\n",asctime(timeInfo));
	// Write column legend
	fprintf(state->file,"#x,y,t,p,u,v\n");

	// Start output handler thread (ONLY IF NECESSARY)
	if (state->mode != OF_OUT_BOTH ||
			atomic_load_explicit(&state->running, memory_order_relaxed)) {
		state->thread = 0;
		if (thrd_create(&state->thread, &outputHandlerThread, state) != thrd_success) {
			caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE,
					"Failed to start output handler thread");
			return (false);
		}
		state->buffer = ringBufferInit(bufferSize);
		if (state->buffer == NULL) {
			caerLog(CAER_LOG_ERROR, SUBSYSTEM_FILE,
					"Failed to allocate transfer ring-buffer.");
			return (false);
		}
		atomic_store(&state->running, true);
	}
	caerLog(CAER_LOG_NOTICE, SUBSYSTEM_FILE, "Logging events to fileName %s",fileName);
	return (true);
}

void closeUartOutput(flowOutputState state) {
	state->running = false;
	uart_close();

	if ((errno = thrd_join(state->thread, NULL)) != thrd_success) {
		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, SUBSYSTEM_UART,
				"Failed to join output handling thread. Error: %d.", errno);
	}
	ringBufferFree(state->buffer);
}


void closeFileOutput(flowOutputState state) {
	if (state->mode != OF_OUT_BOTH) {
		state->running = false;
		if ((errno = thrd_join(state->thread, NULL)) != thrd_success) {
			// This should never happen!
			caerLog(CAER_LOG_CRITICAL, SUBSYSTEM_UART,
					"Failed to join output handling thread. Error: %d.", errno);
		}
		ringBufferFree(state->buffer);
	}
	fclose(state->file);
}

void addPacketToTransferBuffer(flowOutputState state, FlowEventPacket packet) {
	if (packet == NULL) {
		// There was nothing in this mainloop run: return
		return;
	}

	caerEventPacketHeader header = &packet->packetHeader;

	// If no valid events are present, there is nothing to add
	if (caerEventPacketHeaderGetEventValid(header) == 0) {
		return;
	}

	caerEventPacketHeader copy = caerCopyEventPacketOnlyEvents(header);

	if (copy == NULL) {
			caerLog(CAER_LOG_ERROR, SUBSYSTEM_UART,"Failed to copy event packet.");
		return;
	}

	if (!ringBufferPut(state->buffer, copy)) {
		free(copy);
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART,"Failed to add event packet to ring buffer.");
		return;
	}
}

static inline caerEventPacketHeader getPacketFromTransferBuffer(RingBuffer buffer) {
	caerEventPacketHeader packet = ringBufferGet(buffer);
	repeat: if (packet != NULL) {
		// Are there others? Only render last one, to avoid getting backed up!
		caerEventPacketHeader packet2 = ringBufferGet(buffer);

		if (packet2 != NULL) {
			free(packet);
			packet = packet2;
			goto repeat;
		}
	}
	return (packet);
}

static inline bool sendFlowEventPacketUart(caerEventPacketHeader header) {

	FlowEventPacket flow = (FlowEventPacket) header;
	int32_t numValid = caerEventPacketHeaderGetEventValid(header);
	if (numValid == 0) {
		// No events to send - return
		return (false);
	}

	// Send header information for verification. First, two character sequence.
	char* startCallsign = "s";
	char* endCallsign = "\0";
	if (uart_tx((int) strlen(startCallsign), (unsigned char*)startCallsign)) {
		caerLog(CAER_LOG_ERROR,SUBSYSTEM_UART,"Packet start callsign not sent.");
		return (false);
	}
	if (uart_tx(sizeof(startCallsign), (unsigned char*) &numValid)) {
			caerLog(CAER_LOG_ERROR,SUBSYSTEM_UART,"Packet info not sent.");
			return (false);
	}

	// Now send packet content
	for (int32_t i = 0; i < caerEventPacketHeaderGetEventNumber(header); i++) {
		FlowEvent e = &(flow->events[i]);
		if (e == NULL) {
			caerLog(CAER_LOG_ERROR,SUBSYSTEM_UART,"Event null pointer found.");
			return (false);
		}
		if (caerPolarityEventIsValid((caerPolarityEvent) e)) {continue;}
		if (!e->hasFlow) {continue;}
		uint8_t x = (uint8_t) caerPolarityEventGetX((caerPolarityEvent) e);
		uint8_t y = (uint8_t) caerPolarityEventGetY((caerPolarityEvent) e);
		int32_t t = caerPolarityEventGetTimestamp((caerPolarityEvent) e);
		float u = (float) e->u;
		float v = (float) e->v;

		// Send data over UART
		if (uart_tx(sizeof(x),(unsigned char*) &x)
				|| uart_tx(sizeof(y),(unsigned char*) &y)
				|| uart_tx(sizeof(t),(unsigned char*) &t)
				|| uart_tx(sizeof(u),(unsigned char*) &u)
				|| uart_tx(sizeof(v),(unsigned char*) &v))  {
			caerLog(CAER_LOG_ERROR,SUBSYSTEM_UART,"Event not sent.");
			return (false);
		}
	}
	// Finish packet with terminator
	if (uart_tx((int) strlen(endCallsign), (unsigned char*) endCallsign)) {
		caerLog(CAER_LOG_ERROR,SUBSYSTEM_UART,"Packet end callsign not sent.");
		return (false);
	}
	return (true);
}

static inline bool writeFlowEventPacketFile(flowOutputState state,
		caerEventPacketHeader packet) {

	FlowEventPacket flow = (FlowEventPacket) packet;
	if (caerEventPacketHeaderGetEventValid(packet) == 0) {
		return (false);
	}

	for (int32_t i = 0; i < caerEventPacketHeaderGetEventNumber(packet); i++) {
		FlowEvent e = &(flow->events[i]);
		if (e == NULL) {
			caerLog(CAER_LOG_ERROR,SUBSYSTEM_FILE,"Event null pointer found.");
			return (false);
		}
		if (caerPolarityEventIsValid((caerPolarityEvent) e)) {continue;}
		if (!e->hasFlow) {continue;}
		// Get event data
		uint8_t x = (uint8_t) caerPolarityEventGetX((caerPolarityEvent) e);
		uint8_t y = (uint8_t) caerPolarityEventGetY((caerPolarityEvent) e);
		int32_t t = caerPolarityEventGetTimestamp((caerPolarityEvent) e);
		bool p = caerPolarityEventGetPolarity((caerPolarityEvent) e);
		double u = e->u;
		double v = e->v;
		// Write to file
		fprintf(state->file,"%3i,%3i,%10i,%d,%4.3f,%4.3f\n",x,y,t,p,u,v);
	}
	return (true);
}

static int outputHandlerThread(void *stateArg) {
	if (stateArg == NULL) {
		return (thrd_error);
	}
	flowOutputState state = stateArg;
	switch (state->mode) {
		case OF_OUT_UART:
			thrd_set_name(SUBSYSTEM_UART);
			break;
		case OF_OUT_FILE:
			thrd_set_name(SUBSYSTEM_FILE);
			break;
		case OF_OUT_BOTH:
			thrd_set_name("FLOW_OUTPUT");
			break;
		case OF_OUT_NONE:
			break;
	}

	struct timespec sleepTime = { .tv_sec = 0, .tv_nsec = 500000 };

	// Wait until the buffer is initialized
	while (!atomic_load_explicit(&state->running, memory_order_relaxed)) {
		thrd_sleep(&sleepTime, NULL);
	}

	// Main thread loop
	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {
		caerEventPacketHeader packet = getPacketFromTransferBuffer(state->buffer);
		if (packet == NULL) { // no data: sleep for a while
			thrd_sleep(&sleepTime, NULL);
		}
		else {
			if (state->mode == OF_OUT_UART || state->mode == OF_OUT_BOTH) {
				if (!sendFlowEventPacketUart(packet)) {
					caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART, "A flow packet was not sent.");
				}
			}
			if (state->mode == OF_OUT_FILE || state->mode == OF_OUT_BOTH) {
				if (!writeFlowEventPacketFile(state,packet)) {
					caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE, "A flow packet was not written.");
				}
			}
			free(packet);
		}
	}

	// If shutdown: empty buffer before closing thread
	caerEventPacketHeader packet;
	while ((packet = getPacketFromTransferBuffer(state->buffer)) != NULL) {
		if (state->mode == OF_OUT_UART || state->mode == OF_OUT_BOTH) {
			if (!sendFlowEventPacketUart(packet)) {
				caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART, "A flow packet was not sent.");
			}
		}
		if (state->mode == OF_OUT_FILE || state->mode == OF_OUT_BOTH) {
			if (!writeFlowEventPacketFile(state,packet)) {
				caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE, "A flow packet was not written.");
			}
		}
		free(packet);
	}

	return (thrd_success);
}
