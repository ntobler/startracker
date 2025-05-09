/*
 * rpi_protocol.h
 *
 *  Created on: Nov 12, 2023
 *      Author: ftobler
 */

#ifndef RPI_PROTOCOL_H_
#define RPI_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "out.h"


Status* rpi_status();
Acknowledge* rpi_setSettings(Settings* settings);
Trajectory * rpi_calcTrajectory();
Acknowledge* rpi_shutdown();


#ifdef __cplusplus
}
#endif

#endif /* RPI_PROTOCOL_H_ */
