#ifndef FLOWEVENT_H_
#define FLOWEVENT_H_

#include <math.h>
#include <stdbool.h>
#include "main.h"
#include <libcaer/events/polarity.h>

struct flow_event {
	uint16_t x,y;
	int64_t timestamp;
	bool p;
	double u,v;
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
};

typedef struct flow_event_buffer *FlowEventBuffer;

static inline struct flow_event flowEventInit(uint16_t x, uint16_t y, int64_t t, bool p) {
	struct flow_event e;
	e.x = x;
	e.y = y;
	e.timestamp = t;
	e.p = p;
	e.u = 0;
	e.v = 0;
	e.hasFlow = false;
	return (e);
}

static inline struct flow_event flowEventInitFromPolarity(caerPolarityEvent polarity, caerPolarityEventPacket packet) {
	uint16_t x = caerPolarityEventGetX(polarity);
	uint16_t y = caerPolarityEventGetY(polarity);
	int64_t  t = caerPolarityEventGetTimestamp64(polarity,packet);
	bool 	 p = caerPolarityEventGetPolarity(polarity);
	return (flowEventInit(x,y,t,p));
}

static inline FlowEventPacket flowEventPacketInitFromPolarity(caerPolarityEventPacket polarity) {
	if (polarity->packetHeader.eventNumber == 0) {
		return NULL;
	}
	FlowEventPacket flow = malloc(sizeof(FlowEventPacket));
	uint64_t eventNumber = (uint64_t) polarity->packetHeader.eventNumber;
	flow->packetHeader = polarity->packetHeader; //take same specs of packets
	flow->events = malloc(eventNumber * sizeof(struct flow_event));
	uint64_t i;
	for (i=0; i < eventNumber; i++) {
		caerPolarityEvent p = caerPolarityEventPacketGetEvent(polarity,i);
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
				buffer->buffer [i][j][k] = flowEventInit(i,j,0,false);
			}
		}
	}
	return (buffer);
}

static inline void flowEventBufferAdd(struct flow_event e, FlowEventBuffer buffer) {
	// TODO make buffer more efficient for larger buffer size
	size_t i;
	for (i = buffer->size-1; i > 0 ; i--) {
		buffer->buffer[e.x][e.y][i] = buffer->buffer[e.x][e.y][i-1];
	}
	buffer->buffer[e.x][e.y][0] = e;
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
