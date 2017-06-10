#include "file.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "output_common.h"
#include "ext/portable_misc.h"
#include <fcntl.h>
#include <time.h>

#if __APPLE__
#include <sys/syslimits.h>
#endif


static bool caerOutputFileInit(caerModuleData moduleData);

static struct caer_module_functions caerOutputFileFunctions = { .moduleInit = &caerOutputFileInit, .moduleRun =
	&caerOutputCommonRun, .moduleConfig = NULL, .moduleExit = &caerOutputCommonExit, .moduleReset =
	&caerOutputCommonReset };

void caerOutputFile(uint16_t moduleID, size_t outputTypesNumber, ...) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "FileOutput", CAER_MODULE_OUTPUT);
	if (moduleData == NULL) {
		return;
	}

	va_list args;
	va_start(args, outputTypesNumber);
	caerModuleSMv(&caerOutputFileFunctions, moduleData, CAER_OUTPUT_COMMON_STATE_STRUCT_SIZE, outputTypesNumber, args);
	va_end(args);
}

static char *getUserHomeDirectory(const char *subSystemString);
static char *getFullFilePath(const char *subSystemString, const char *directory, const char *prefix);

// Remember to free strings returned by this.
static char *getUserHomeDirectory(const char *subSystemString) {
	size_t homeDirLength = PATH_MAX;

	// Allocate memory for home directory path.
	char *homeDir = malloc(homeDirLength);
	if (homeDir == NULL) {
		caerLog(CAER_LOG_ERROR, subSystemString, "Failed to allocate memory for home directory string.");
		return (NULL);
	}

	// Discover home directory path, use libuv for cross-platform support.
	int retVal = uv_os_homedir(homeDir, &homeDirLength);
	UV_RET_CHECK(retVal, subSystemString, "uv_os_homedir", return (NULL));

	return (homeDir);
}

static char *getFullFilePath(const char *subSystemString, const char *directory, const char *prefix) {
	// First get time suffix string.
	time_t currentTimeEpoch = time(NULL);

#if defined(OS_WINDOWS)
	// localtime() is thread-safe on Windows (and there is no localtime_r() at all).
	struct tm *currentTime = localtime(&currentTimeEpoch);
#else
	// From localtime_r() man-page: "According to POSIX.1-2004, localtime()
	// is required to behave as though tzset(3) was called, while
	// localtime_r() does not have this requirement."
	// So we make sure to call it here, to be portable.
	tzset();

	struct tm currentTimeStruct;
	struct tm *currentTime = &currentTimeStruct;

	localtime_r(&currentTimeEpoch, currentTime);
#endif

	// Following time format uses exactly 19 characters (5 separators,
	// 4 year, 2 month, 2 day, 2 hours, 2 minutes, 2 seconds).
	size_t currentTimeStringLength = 19;
	char currentTimeString[currentTimeStringLength + 1]; // + 1 for terminating NUL byte.
	strftime(currentTimeString, currentTimeStringLength + 1, "%Y_%m_%d_%H_%M_%S", currentTime);

	if (caerStrEquals(prefix, "")) {
		// If the prefix is the empty string, use a minimal one.
		prefix = DEFAULT_PREFIX;
	}

	// Assemble together: directory/prefix-time.aedat
	size_t filePathLength = strlen(directory) + strlen(prefix) + currentTimeStringLength + 9;
	// 1 for the directory/prefix separating slash, 1 for prefix-time separating
	// dash, 6 for file extension, 1 for terminating NUL byte = +9.

	char *filePath = malloc(filePathLength);
	if (filePath == NULL) {
		caerLog(CAER_LOG_CRITICAL, subSystemString, "Unable to allocate memory for full file path.");
		return (NULL);
	}

	snprintf(filePath, filePathLength, "%s/%s-%s.aedat", directory, prefix, currentTimeString);

	return (filePath);
}

static bool caerOutputFileInit(caerModuleData moduleData) {
	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	char *logDir = getLogPath(moduleData->moduleSubSystemString);
	if (logDir == NULL) {
		// caerLog() called inside getUserHomeDirectory().
		return (false);
	}

	sshsNodePutStringIfAbsent(moduleData->moduleNode, "directory", logDir);
	free(logDir);

	sshsNodePutStringIfAbsent(moduleData->moduleNode, "prefix", DEFAULT_PREFIX);

	// Generate current file name and open it.
	char *directory = sshsNodeGetString(moduleData->moduleNode, "directory");
	char *prefix = sshsNodeGetString(moduleData->moduleNode, "prefix");

	char *filePath = getFullFilePath(moduleData->moduleSubSystemString, directory, prefix);
	free(directory);
	free(prefix);

	if (filePath == NULL) {
		// caerLog() called inside getFullFilePath().
		return (false);
	}

	int fileFd = open(filePath, O_WRONLY | O_CREAT, S_IWUSR | S_IRUSR | S_IRGRP);
	if (fileFd < 0) {
		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Could not create or open output file '%s' for writing. Error: %d.", filePath, errno);
		free(filePath);

		return (false);
	}

	caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "Opened output file '%s' successfully for writing.",
		filePath);
	free(filePath);

	if (!caerOutputCommonInit(moduleData, fileFd, NULL)) {
		close(fileFd);

		return (false);
	}

	return (true);
}
