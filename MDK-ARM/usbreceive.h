#ifndef __USBRECEIVE_H__
#define __USBRECEIVE_H__

#include <string.h>
#include "dm4310_drv.h"
#include "dm4310_ctrl.h"
#include "usbd_cdc_if.h"
#include "delay.h"

void receive_data_control(uint16_t *received_number, motor_t *motor);

#endif

