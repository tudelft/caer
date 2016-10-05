#ifndef FLOWEVENT_H_
#define FLOWEVENT_H_

#include <math.h>
#include <stdbool.h>
#include "main.h"
#include "ext/buffers.h"
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
 * - xu,yu, the undistorted pixel coordinates
 * - hasFlow, a flag indicating that flow has been assigned to this event.
 */
struct flow_event {
	uint32_t data;
	int64_t timestamp;
	float u,v;
	float xu,yu;
	bool hasFlow;
}__attribute__((__packed__));

/**
 * Pointer to flow event.
*/
typedef struct flow_event *flowEvent;

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
typedef struct flow_event_packet *flowEventPacket;

/**
 * Map of pixel coordinates to undistorted pixel coordinates (FLOATS instead of uint8_t)
 */
struct float_array {
	float **array;
	size_t sizeX;
	size_t sizeY;
};

typedef struct float_array *floatArray;

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
	e.xu = 0;
	e.yu = 0;
	e.hasFlow = false;
	return (e);
}

/**
 *  Flow event initialization from an existing polarity event.
*/
static inline struct flow_event flowEventInitFromPolarity(caerPolarityEvent polarity,
		caerPolarityEventPacket packet) {
	uint32_t data = polarity->data;
	int64_t  timestamp = caerPolarityEventGetTimestamp64(polarity,packet);
	return (flowEventInit(data,timestamp));
}

/**
 *  Flow event initialization from x,y coordinates, timestamp, and polarity.
*/
static inline struct flow_event flowEventInitXYTP(uint16_t x, uint16_t y, int64_t t, bool p) {
	struct flow_event e;
	caerPolarityEventSetX((caerPolarityEvent) &e, x);
	caerPolarityEventSetY((caerPolarityEvent) &e, y);
	caerPolarityEventSetPolarity((caerPolarityEvent) &e, (_Bool) p);
	SET_NUMBITS32(e.data, VALID_MARK_SHIFT, VALID_MARK_MASK, 1); // validate event
	e.timestamp = t;
	e.u = 0;
	e.v = 0;
	e.xu = 0;
	e.yu = 0;
	e.hasFlow = false;
	return (e);
}

static inline uint8_t flowEventGetX(flowEvent event) {
	return U8T(GET_NUMBITS32(event->data, X_ADDR_SHIFT, X_ADDR_MASK));
}
static inline uint8_t flowEventGetY(flowEvent event) {
	return U8T(GET_NUMBITS32(event->data, Y_ADDR_SHIFT,Y_ADDR_MASK));
}
static inline bool flowEventGetPolarity(flowEvent event) {
	return (GET_NUMBITS32(event->data, POLARITY_SHIFT, POLARITY_MASK));
}
static inline bool flowEventIsValid(flowEvent event) {
	return (GET_NUMBITS32(event->data, VALID_MARK_SHIFT, VALID_MARK_MASK));
}

/**
 * A flow event packet is for now initialized by copying the content of an
 * existing polarity event packet. This prevents having to adapt the original
 * libcaer definitions, while enabling packet management using existing methods.
 */
static inline flowEventPacket flowEventPacketInitFromPolarity(caerPolarityEventPacket polarity) {
	if (polarity == NULL) {
		return NULL;
	}
	if (polarity->packetHeader.eventNumber == 0) {
		return NULL;
	}

	uint64_t eventNumber = (uint64_t) polarity->packetHeader.eventNumber;
	size_t eventCapacity = (size_t) polarity->packetHeader.eventCapacity;
	size_t eventSize = sizeof(struct flow_event);
	flowEventPacket flow = malloc(sizeof(struct flow_event_packet) + eventCapacity*eventSize);

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
static inline flowEvent flowEventPacketGetEvent(flowEventPacket packet, int32_t n) {
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
 * Make a copy of a flow event packet.
*/
static inline flowEventPacket flowEventPacketCopy(flowEventPacket flow) {
	// Handle empty event packets.
	if (flow == NULL) {
		return (NULL);
	}

	// Calculate needed memory for new event packet.
	caerEventPacketHeader header = &(flow->packetHeader);
	int32_t eventSize = caerEventPacketHeaderGetEventSize(header);
	int32_t eventNumber = caerEventPacketHeaderGetEventNumber(header);
	int32_t eventValid = caerEventPacketHeaderGetEventValid(header);

	if (eventValid == 0) {
		// No copy possible if result is empty (capacity=0).
		return (NULL);
	}

	size_t packetMem = CAER_EVENT_PACKET_HEADER_SIZE + (size_t) (eventSize * eventValid);

	// Allocate memory for new event packet.
	flowEventPacket eventPacketCopy = (flowEventPacket) malloc(packetMem);
	if (eventPacketCopy == NULL) {
		// Failed to allocate memory.
		return (NULL);
	}

	// First copy over the header.
	memcpy(eventPacketCopy, flow, CAER_EVENT_PACKET_HEADER_SIZE);

	// Copy the data over. Must check every event for validity!
	size_t offset = CAER_EVENT_PACKET_HEADER_SIZE;

	int i;
	for (i = 0; i < eventNumber; i++) {
		flowEvent e = flowEventPacketGetEvent(flow, i);
		if (caerPolarityEventIsValid((caerPolarityEvent) e)) continue;
		memcpy(((uint8_t *) eventPacketCopy) + offset, e, (size_t) eventSize);
		offset += (size_t) eventSize;
	}

	// Set the event capacity and the event number to the number of
	// valid events, since we only copied those.
	caerEventPacketHeaderSetEventCapacity(&(eventPacketCopy->packetHeader), eventValid);
	caerEventPacketHeaderSetEventNumber(&(eventPacketCopy->packetHeader), eventValid);

	return (eventPacketCopy);
}

/**
 * Free flow event packet memory.
*/
static inline void flowEventPacketFree(flowEventPacket flow) {
	if (flow != NULL) {
		free(flow->events);
		free(flow);
	}
	flow = NULL; // make sure that free is not called twice on the same packet
}

/**
 * Set interface to cAER's simple2DBufferLong type
 */
static inline bool simple2DBufferLongSet(simple2DBufferLong buffer, size_t x, size_t y, int64_t value) {
	if (x >= buffer->sizeX || y >= buffer->sizeY)
		return (false);
	buffer->buffer2d[x][y] = value;
	return (true);
}

/**
 * Get interface to cAER's simple2DBufferLong type
 */
static inline int64_t simple2DBufferLongGet(simple2DBufferLong buffer, size_t x, size_t y) {
	if (x >= buffer->sizeX || y >= buffer->sizeY)
		return (0);
	return (buffer->buffer2d[x][y]);
}

#endif
