#ifndef INPUT_OUTPUT_COMMON_H_
#define INPUT_OUTPUT_COMMON_H_

#include "main.h"
#include <libcaer/network.h>
#include <limits.h>

static inline void caerGenericEventSetTimestamp(void *eventPtr, caerEventPacketHeader headerPtr, int32_t timestamp) {
	*((int32_t *) (((uint8_t *) eventPtr) + U64T(caerEventPacketHeaderGetEventTSOffset(headerPtr)))) = htole32(
		timestamp);
}

// Return the path to the caer executable
// Remember to free strings returned by this.
static inline char *getTopPath(const char *subSystemString) {
	// Allocate memory for home directory path.
	char *dir = malloc(PATH_MAX);
	if (dir == NULL) {
		caerLog(CAER_LOG_ERROR, subSystemString, "Failed to allocate memory for log directory string.");
		return (NULL);
	}

	// get path to head folder
	char symlink_path[PATH_MAX];
	pid_t pid = getpid();
	sprintf(symlink_path, "/proc/%d/exe", pid);
	if (readlink(symlink_path, dir, PATH_MAX) == -1){
		caerLog(CAER_LOG_ERROR, subSystemString, "Failed to get path to main directory string.");
		return (NULL);
	}

	// strip function name
	char* fn = strrchr(dir,'/caer-bin');
	if (fn)	{
		*(fn-8) = '\0';
	}

	return (dir);
}

// Returns path of log folder
// Remember to free strings returned by this.
static inline char *getLogPath(const char *subSystemString) {
	// Allocate memory for home directory path.
	char *logDir = getTopPath(subSystemString);
	strcat(logDir, "/logs");

	return (logDir);
}

#endif /* INPUT_OUTPUT_COMMON_H_ */
