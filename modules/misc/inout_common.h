#ifndef INPUT_OUTPUT_COMMON_H_
#define INPUT_OUTPUT_COMMON_H_

#include "main.h"

#define AEDAT3_NETWORK_HEADER_LENGTH 20
#define AEDAT3_NETWORK_MAGIC_NUMBER 0x1D378BC90B9A6658
#define AEDAT3_NETWORK_VERSION 0x01
#define AEDAT3_FILE_VERSION "3.1"

#define MAX_OUTPUT_UDP_SIZE (1472 - AEDAT3_NETWORK_HEADER_LENGTH) // MTU 1500 - 20 IP header - 8 UDP header

PACKED_STRUCT(struct aedat3_network_header {
	int64_t magicNumber;
	int64_t sequenceNumber;
	int8_t versionNumber;
	int8_t formatNumber;
	int16_t sourceID;
});

static inline struct aedat3_network_header caerParseNetworkHeader(uint8_t *dataBuffer) {
	// Network header is 20 bytes long. Use struct to interpret.
	struct aedat3_network_header networkHeader;

	// Copy data into packet struct.
	memcpy(&networkHeader, dataBuffer, AEDAT3_NETWORK_HEADER_LENGTH);

	// Ensure endianness conversion is done if needed.
	networkHeader.magicNumber = le64toh(networkHeader.magicNumber);
	networkHeader.sequenceNumber = le64toh(networkHeader.sequenceNumber);
	networkHeader.sourceID = le16toh(networkHeader.sourceID);

	return (networkHeader);
}

static inline void caerGenericEventSetTimestamp(void *eventPtr, caerEventPacketHeader headerPtr, int32_t timestamp) {
	*((int32_t *) (((uint8_t *) eventPtr) + U64T(caerEventPacketHeaderGetEventTSOffset(headerPtr)))) = htole32(
		timestamp);
}

#endif /* INPUT_OUTPUT_COMMON_H_ */
