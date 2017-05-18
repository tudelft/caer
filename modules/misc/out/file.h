#ifndef OUTPUT_FILE_H_
#define OUTPUT_FILE_H_

#include "main.h"

#define DEFAULT_PREFIX "caerOut"

char *getLogPath(const char *subSystemString);
char *getTopPath(const char *subSystemString);

void caerOutputFile(uint16_t moduleID, size_t outputTypesNumber, ...);

#endif /* OUTPUT_FILE_H_ */
