#ifndef __USBSENT_H__
#define __USBSENT_H__

#include "dm4310_drv.h"
#include "dm4310_ctrl.h"

#define DATASIZE 50
#define StartByte 0xAB
#define EndByte 0xBA

void send_data(motor_t *motor);

#endif
