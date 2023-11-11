/*
 * virtual_com.h
 *
 *  Created on: Nov 7, 2023
 *      Author: cchtofl01
 */

#ifndef SRC_VIRTUAL_COM_H_
#define SRC_VIRTUAL_COM_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "stdint.h"

void vcp_put_data_usb_driver(uint8_t* buf, uint32_t len);

#ifdef __cplusplus
}
#endif


#endif /* SRC_VIRTUAL_COM_H_ */
