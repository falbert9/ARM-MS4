#ifndef CONDUCTOR_H
#define CONDUCTOR_H
#include "vtI2C.h"
#include "navigation.h"
#include "mapping.h"
#include "distance.h"
// Structure used to pass parameters to the task
// Do not touch...
typedef struct __ConductorStruct {
	vtI2CStruct *dev;
	vtNavStruct *navData;
	vtMapStruct *mapData;
	vtDistanceStruct *distanceData;
} vtConductorStruct;

// Public API
//
// The job of this task is to read from the message queue that is output by the I2C thread and to distribute the messages to the right
//   threads.  Right now, they are only going to one thread, but it should be clear from the structure of this task how that would be done.
// Start the task
// Args:
//   conductorData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   i2c: pointer to the data structure for an i2c task
//   navigation: pointer to the data structure for an navigation task
//	 mapping: pointer to the data structure for a mapping task
void vStartConductorTask(vtConductorStruct *conductorData,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtNavStruct *navigation, vtMapStruct *mapping, vtDistanceStruct *distance);
#endif