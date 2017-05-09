/*
 * uartOutput.c
 *
 *  Created on: Jul 5, 2016
 *      Author: bas
 */

#include "flowOutput.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

#define SUBSYSTEM_UART "UART"
#define SUBSYSTEM_FILE "Event logger"
#define FILE_MAX_NUMBER_OF_LINES 5000000

void addFlowEventToTransferBuffer(flowOutputState state, flowEvent e) {
	if (e == NULL) {
		// event transmisison problem - nothing to copy
		return;
	}
	if (!e->hasFlow) {
		// only copy event with flow
		return;
	}

	// Copy event content
	flowEvent copy = malloc(sizeof(struct flow_event));
	*copy = *e;

	// Add to ring buffer
	if (!ringBufferPut(state->buffer, copy)) {
		free(copy);
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART,"Failed to add event to ring buffer.");
		return;
	}
}

static flowEvent getFlowEventFromTransferBuffer(RingBuffer buffer) {
	flowEvent e = ringBufferGet(buffer);
	return (e);
}

static bool sendFlowEventUart(flowEvent e) {
	if (e == NULL) {
		caerLog(CAER_LOG_ERROR,SUBSYSTEM_UART,"Event null pointer found.");
		return (false);
	}
	if (!e->hasFlow) {
		return (false);
	}
	// Events are separated by unsigned ints of value 255. This value should
	// never occur as pixel location
	unsigned char eventSeparator = 255;

	int16_t x = (int16_t) (e->xu*10);
	int16_t y = (int16_t) (e->yu*10);
	int32_t t = caerPolarityEventGetTimestamp((caerPolarityEvent) e);
	int16_t u = (int16_t) (e->u*10);
	int16_t v = (int16_t) (e->v*10);

	// Send data over UART
	//uart_drain();
	if (uart_tx(sizeof(x),(unsigned char*) &x)
			|| uart_tx(sizeof(y),(unsigned char*) &y)
			|| uart_tx(sizeof(t),(unsigned char*) &t)
			|| uart_tx(sizeof(u),(unsigned char*) &u)
			|| uart_tx(sizeof(v),(unsigned char*) &v)
			|| uart_tx(sizeof(eventSeparator), &eventSeparator))  {
		int errnum = errno;
		caerLog(CAER_LOG_ERROR,SUBSYSTEM_UART,"TX error: %s",strerror(errnum));
		//exit(EXIT_FAILURE); // Prevent overflow of errors
		return (false);
	}
	return (true);
}

static bool writeFlowEventToFile(flowOutputState state,
		flowEvent e) {

	if (e == NULL) {
		caerLog(CAER_LOG_ERROR,SUBSYSTEM_FILE,"Event null pointer found.");
		return (false);
	}
	if (!e->hasFlow) {
		return (false);
	}

	// Get event data
	uint8_t x = (uint8_t) caerPolarityEventGetX((caerPolarityEvent) e);
	uint8_t y = (uint8_t) caerPolarityEventGetY((caerPolarityEvent) e);
	int32_t t = caerPolarityEventGetTimestamp((caerPolarityEvent) e);
	bool p = caerPolarityEventGetPolarity((caerPolarityEvent) e);
	double u = e->u;
	double v = e->v;

	// Write to file
	if (state->fileLineNumber < FILE_MAX_NUMBER_OF_LINES)  {
		fprintf(state->file,"%3i,%3i,%10i,%d,%4.3f,%4.3f\n",x,y,t,p,u,v);
		state->fileLineNumber++;
	}
	else {
		// If too many lines, stop logging to prevent overrun
		if (state->fileLineNumber == FILE_MAX_NUMBER_OF_LINES) {
			caerLog(CAER_LOG_NOTICE, SUBSYSTEM_FILE,
					"File log reached limit of %d lines - "
					"no more lines will be added.", FILE_MAX_NUMBER_OF_LINES);
			state->fileLineNumber++;
		}
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

	struct timespec sleepTime = { .tv_sec = 0, .tv_nsec = 100000 };
	struct timespec sleepTime2 = { .tv_sec = 0, .tv_nsec = 100000 };

	// Wait until the buffer is initialized
	while (!atomic_load_explicit(&state->running, memory_order_relaxed)) {
		thrd_sleep(&sleepTime, NULL);
	}

	// Main thread loop
	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {
		flowEvent e = getFlowEventFromTransferBuffer(state->buffer);
		if (e != NULL) {
			if (state->mode == OF_OUT_UART || state->mode == OF_OUT_BOTH) {
				if (!sendFlowEventUart(e)) {
					caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART, "A flow event was not sent.");
				}
			}
			if (state->mode == OF_OUT_FILE || state->mode == OF_OUT_BOTH) {
				if (!writeFlowEventToFile(state,e)) {
					caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE, "A flow event was not written.");
				}
			}
			free(e);
		} else {
			thrd_sleep(&sleepTime,NULL);	
		}
		thrd_sleep(&sleepTime2,NULL);
	}

	// If shutdown: empty buffer before closing thread
	flowEvent e;
	while ((e = getFlowEventFromTransferBuffer(state->buffer)) != NULL) {
		if (state->mode == OF_OUT_UART || state->mode == OF_OUT_BOTH) {
			if (!sendFlowEventUart(e)) {
				caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART, "A flow event was not sent.");
			}
		}
		if (state->mode == OF_OUT_FILE || state->mode == OF_OUT_BOTH) {
			if (!writeFlowEventToFile(state,e)) {
				caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE, "A flow event was not written.");
			}
		}
		free(e);
	}

	return (thrd_success);
}

bool initUartOutput(flowOutputState state, const char* port, unsigned int baud, size_t bufferSize) {
	// Initialize UART communication
	int uartErrno = uart_open(port, baud);
	if (uartErrno) {
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART,
				"Failed to identify serial communication, errno=%i.",uartErrno);
		return (false);
	}
	// Test message for sync at startup
	unsigned char testMessage = 255;
	if (uart_tx(1, &testMessage)) {
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

bool initFileOutput(flowOutputState state, const char* fileBaseName, size_t bufferSize) {
	// Get timestamp info
	time_t rawTime;
	time (&rawTime);
	const struct tm * timeInfo = localtime(&rawTime);
	char fileName[128];
	char fileTimestamp[64];
	strftime(fileTimestamp, sizeof(fileTimestamp),"%Y_%m_%d_%H_%M_%S", timeInfo);
	sprintf(fileName, "%s_%s.csv", fileBaseName, fileTimestamp);

	// Check if filename exists
	struct stat st;
	int n = 0;
	while (stat(fileName,&st) == 0) {
		caerLog(CAER_LOG_WARNING, SUBSYSTEM_FILE,
				"Filename %s is already used.", fileName);
		n++;
		sprintf(fileName, "%s_%s_%d.csv", fileBaseName, fileTimestamp, n);
	}

	// Initialize file communication
	state->file = fopen(fileName,"w+");
	if (state->file == NULL) {
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE,"Failed to open file %s", fileName);
		return (false);
	}
	// Write header as check for file output
	if (fprintf(state->file,"#cAER event data with optic flow values\n") < 0) {
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE,"Failed to write header");
		fclose(state->file);
		return (false);
	}
	// Write creation date
	fprintf(state->file,"#Date created: %s\n",asctime(timeInfo));
	// Write column legend
	fprintf(state->file,"#x,y,t,p,u,v\n");

	state->fileLineNumber = 4;

	// Start output handler thread (ONLY IF NOT YET CALLED BY UART)
	if (state->mode != OF_OUT_BOTH ||
			!atomic_load_explicit(&state->running, memory_order_relaxed)) {
		state->mode = OF_OUT_FILE; // to ensure that only file output is performed in this case
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
	caerLog(CAER_LOG_NOTICE, SUBSYSTEM_FILE, "Logging flow events to %s",fileName);

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

