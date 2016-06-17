#ifndef FLOWEVENT_H_
#define FLOWEVENT_H_

#include <math.h>
#include <stdbool.h>
#include "main.h"
#include <libcaer/events/polarity.h>

struct flow_event {
	uint32_t data;
	int64_t timestamp;
	float u,v;
	bool hasFlow;
}__attribute__((__packed__));

typedef struct flow_event *FlowEvent;

struct flow_event_packet {
	/// The common event packet header.
	struct caer_event_packet_header packetHeader;
	/// The events array.
	struct flow_event* events;
}__attribute__((__packed__));

typedef struct flow_event_packet *FlowEventPacket;

struct flow_event_buffer {
	struct flow_event*** buffer;
	size_t sizeX;
	size_t sizeY;
	size_t size;
}__attribute__((__packed__));

typedef struct flow_event_buffer *FlowEventBuffer;

static inline struct flow_event flowEventInit(uint32_t data, int64_t timestamp) {
	struct flow_event e;
	e.data = data;
	e.timestamp = timestamp;
	e.u = 0;
	e.v = 0;
	e.hasFlow = false;
	return (e);
}

static inline struct flow_event flowEventInitFromPolarity(caerPolarityEvent polarity, caerPolarityEventPacket packet) {
	uint32_t data = polarity->data;
	int64_t  timestamp = caerPolarityEventGetTimestamp64(polarity,packet);
	return (flowEventInit(data,timestamp));
}

static inline struct flow_event flowEventInitXYTP(uint16_t x, uint16_t y, int32_t t, bool p) {
	struct flow_event e;
	caerPolarityEventSetX((caerPolarityEvent) &e, x);
	caerPolarityEventSetX((caerPolarityEvent) &e, y);
	caerPolarityEventSetPolarity((caerPolarityEvent) &e, (_Bool) p);
	e.timestamp = t;
	return (e);
}

static inline FlowEventPacket flowEventPacketInitFromPolarity(caerPolarityEventPacket polarity) {
	if (polarity == NULL) {
		return NULL;
	}
	if (polarity->packetHeader.eventNumber == 0) {
		return NULL;
	}

	uint64_t eventNumber = (uint64_t) polarity->packetHeader.eventNumber;
	size_t eventCapacity = (size_t) polarity->packetHeader.eventCapacity;
	size_t eventSize = sizeof(struct flow_event);
	FlowEventPacket flow = malloc(sizeof(struct flow_event_packet) + eventCapacity*eventSize);

//	caerLog(CAER_LOG_CRITICAL, "FLOW: ","Error");
	flow->packetHeader = polarity->packetHeader; //take same specs of packets
	flow->events = malloc(eventNumber * sizeof(struct flow_event));
	uint64_t i;
	for (i=0; i < eventNumber; i++) {
		caerPolarityEvent p = caerPolarityEventPacketGetEvent(polarity,(int)i);
		flow->events[i] = flowEventInitFromPolarity(p, polarity);
	}
	return (flow);
}

static inline FlowEventBuffer flowEventBufferInit(size_t width, size_t height, size_t size) {
	FlowEventBuffer buffer = malloc(sizeof(*buffer) + (width * sizeof(struct flow_event *)));
	buffer->sizeX = width;
	buffer->sizeY = height;
	buffer->size = size;

	buffer->buffer = malloc(width * sizeof(double*));
	uint16_t i,j,k;
	for (i = 0; i < width; i++) {
		buffer->buffer [i] = malloc(height * sizeof(double*));
		for (j = 0; j < height; j++) {
			buffer->buffer [i][j] = malloc(size * sizeof(struct flow_event));
			for (k = 0; k < size; k++) {
				buffer->buffer [i][j][k] = flowEventInitXYTP(i,j,0,false);
			}
		}
	}
	return (buffer);
}

static inline void flowEventBufferAdd(struct flow_event e, FlowEventBuffer buffer) {
	// TODO make buffer more efficient for larger buffer size
	size_t i;
	uint16_t x;
	uint16_t y;
	for (i = buffer->size-1; i > 0 ; i--) {
		x = caerPolarityEventGetX((caerPolarityEvent) &e);
		y = caerPolarityEventGetY((caerPolarityEvent) &e);
		buffer->buffer[x][y][i] = buffer->buffer[x][y][i-1];
	}
	buffer->buffer[x][y][0] = e;
}

static inline void flowEventPacketFree(FlowEventPacket flow) {
	if (flow != NULL) {
		free(flow->events);
		free(flow);
	}
}

static inline void flowEventBufferFree(FlowEventBuffer buffer) {
	if (buffer != NULL) {
		uint16_t i,j;
		for (i = 0; i < buffer->sizeX; i++) {
			for (j = 0; j < buffer->sizeY; j++) {
				free(buffer->buffer[i][j]);
			}
			free(buffer->buffer[i]);
		}
		free(buffer->buffer);
		free(buffer);
	}
}

#endif
