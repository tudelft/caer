#ifndef FLOWEVENT_H_
#define FLOWEVENT_H_

#include <math.h>
#include <stdbool.h>
#include "main.h"
#include <libcaer/events/polarity.h>

#define FLOW_EVENT_TYPE 101

/**
 * The flow event is a custom event type that 'extends' the polarity
 * event type defined in libcaer, having address/timestamp data.
 * Hence, all basic get/set functions for the polarity event work equally
 * for this new event type.
 *
 * Note that the timestamp in this variable is 64 bits.
 *
 * It is augmented with three variables:
 * - u,v, which indicate horizontal/vertical optical flow speed
 * - hasFlow, a flag indicating that flow has been assigned to this event.
 */
struct flow_event {
	uint32_t data;
	int64_t timestamp;
	double u,v;
	bool hasFlow;
}__attribute__((__packed__));

/**
 * Pointer to flow event.
*/
typedef struct flow_event *FlowEvent;

/**
 * Similar to a polarity event packet, a flow event packet contains a
 * packet header, and a container of the events.
 * The flow event packet is hence compatible with existing functions
 * in the visualizer and in libcaer
 */
struct flow_event_packet {
	/// The common event packet header.
	struct caer_event_packet_header packetHeader;
	/// The events array.
	struct flow_event* events;
}__attribute__((__packed__));

/**
 * Pointer to flow event packet.
*/
typedef struct flow_event_packet *FlowEventPacket;

/**
 * The flow event buffer stores full event data for use with optic
 * flow computation algorithms. It stores data not only in x/y, but
 * also allows storing multiple sequential events at a position.
 */
struct flow_event_buffer {
	struct flow_event*** buffer;
	size_t sizeX;
	size_t sizeY;
	size_t size;
}__attribute__((__packed__));

/**
 * Pointer to flow event buffer.
*/
typedef struct flow_event_buffer *FlowEventBuffer;


/**
 *  Flow event initialization. By default, a flow event is not assigned
 *  any optic flow value.
*/
static inline struct flow_event flowEventInit(uint32_t data, int64_t timestamp) {
	struct flow_event e;
	e.data = data;
	e.timestamp = timestamp;
	e.u = 0;
	e.v = 0;
	e.hasFlow = false;
	return (e);
}

/**
 *  Flow event initialization from an existing polarity event.
*/
static inline struct flow_event flowEventInitFromPolarity(caerPolarityEvent polarity, caerPolarityEventPacket packet) {
	uint32_t data = polarity->data;
	int64_t  timestamp = caerPolarityEventGetTimestamp64(polarity,packet);
	return (flowEventInit(data,timestamp));
}

/**
 *  Flow event initialization from x,y coordinates, timestamp, and polarity.
*/
static inline struct flow_event flowEventInitXYTP(uint16_t x, uint16_t y, int32_t t, bool p) {
	struct flow_event e;
	caerPolarityEventSetX((caerPolarityEvent) &e, x);
	caerPolarityEventSetY((caerPolarityEvent) &e, y);
	caerPolarityEventSetPolarity((caerPolarityEvent) &e, (_Bool) p);
	SET_NUMBITS32(e.data, VALID_MARK_SHIFT, VALID_MARK_MASK, 1); // validate event
	e.timestamp = t;
	e.u = 0;
	e.v = 0;
	e.hasFlow = false;
	return (e);
}

/**
 * A flow event packet is for now initialized by copying the content of an
 * existing polarity event packet. This prevents having to adapt the original
 * libcaer definitions, while enabling packet management using existing methods.
 */
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

	flow->packetHeader = polarity->packetHeader; //take same specs of packets
	flow->events = calloc(eventNumber,sizeof(struct flow_event));
	uint64_t i;
	for (i=0; i < eventNumber; i++) {
		caerPolarityEvent p = caerPolarityEventPacketGetEvent(polarity,(int)i);
		flow->events[i] = flowEventInitFromPolarity(p, polarity);
	}

	// Adapt packetHeader to make the flow event packet compatible with any modifications
	caerEventPacketHeaderSetEventType(&(flow->packetHeader),FLOW_EVENT_TYPE);
	caerEventPacketHeaderSetEventSize(&(flow->packetHeader),sizeof(struct flow_event));

	return (flow);
}

/**
 * Obtain a flow event from a packet, checking the index limits of the packet.
*/
static inline FlowEvent flowEventPacketGetEvent(FlowEventPacket packet, int32_t n) {
	// Check that we're not out of bounds.
	if (n < 0 || n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
		caerLog(CAER_LOG_CRITICAL, "Flow Event",
			"Called flowEventPacketGetEvent() with invalid event offset %" PRIi32
			", while maximum allowed value is %" PRIi32 ".",
			n, caerEventPacketHeaderGetEventCapacity(&packet->packetHeader) - 1);
		return (NULL);
	}

	// Return a pointer to the specified event.
	return (packet->events + n);
}

/**
 * Free flow event packet memory.
*/
static inline void flowEventPacketFree(FlowEventPacket flow) {
	if (flow != NULL) {
		free(flow->events);
		free(flow);
	}
}

/**
 * Memory allocation and initialization of a flow event buffer.
 * The events in the buffer are initialized with zero timestamp and negative polarity.
*/
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

/**
 * Add a flow event event to a buffer, checking the buffer bounds.
 * With each new event, the previously added events on a pixel location
 * are shifted forward in time.
 *
 * Returns true if successful, false if the event cannot be added to the buffer.
*/
static inline bool flowEventBufferAdd(FlowEvent e, FlowEventBuffer buffer) {
	// TODO make buffer update more efficient for larger buffer size,
	// 		e.g. by using a ring buffer structure
	size_t i;
	uint16_t x = caerPolarityEventGetX((caerPolarityEvent) e);
	uint16_t y = caerPolarityEventGetY((caerPolarityEvent) e);

	if (x > buffer->sizeX) {
		caerLog(CAER_LOG_ALERT,"FLOW: ", "Event buffer write access out of bounds: x=%i\n", x);
		return (false);
	}
	if (y > buffer->sizeY) {
		caerLog(CAER_LOG_ALERT,"FLOW: ", "Event buffer write access out of bounds: y=%i\n", y);
		return (false);
	}

	for (i = buffer->size-1; i > 0 ; i--) {
		buffer->buffer[x][y][i] = buffer->buffer[x][y][i-1];
	}
	buffer->buffer[x][y][0] = *e; // copy the event
	return (true);
}

/**
 * Finds the content of a flow event buffer at a specified location, and returns a pointer to the event.
 */
static inline FlowEvent flowEventBufferRead(FlowEventBuffer buffer, uint16_t x, uint16_t y, uint16_t i) {
	if (x > buffer->sizeX) {
		caerLog(CAER_LOG_ALERT,"FLOW: ", "Event buffer read access out of bounds: x=%i\n", x);
		return (NULL);
	}
	if (y > buffer->sizeY) {
		caerLog(CAER_LOG_ALERT,"FLOW: ", "Event buffer read access out of bounds: y=%i\n", y);
		return (NULL);
	}
	if (i > buffer->size) {
		caerLog(CAER_LOG_ALERT,"FLOW: ", "Event buffer read access out of bounds: i=%i\n", i);
		return (NULL);
	}
	return (&(buffer->buffer[x][y][i]));
}

/**
 * Free the memory of a flow event buffer.
 */
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
