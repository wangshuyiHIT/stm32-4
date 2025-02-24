#include <string.h>
#include "dm4310_drv.h"
#include "dm4310_ctrl.h"
#include "usbd_cdc_if.h"
#include "usbreceive.h"
#include "delay.h"

float target_q[12] = {0};

//position limit order = MOTOR   1     2     3     4     5     6     7     8     9     A     B     C
float motor_lower_limit[12]  = {-0.15,-1.0, -1.0, -0.4, -1.0, -0.5, -0.7, -1.2, -1.0, -1.5, -1.0, -0.5};
float motor_higher_limit[12] = {  0.7, 1.2,  1.0,  1.5,  1.0,  0.5, 0.15,  1.0,  1.0,  0.4,  1.0,  0.5};

//float motor_lower_limit[12]  = {-0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1};
//float motor_higher_limit[12] = { 0.1,  0.1,  0.1,  0.1,  0.1,  0.1,  0.1,  0.1,  0.1,  0.1,  0.1,  0.1};

void receive_data_control(uint16_t *received_number, motor_t *motor){

	
	for (int i = 0; i < 12; i++){
	
		target_q[i] = uint_to_float(received_number[i], -12.5, 12.5, 16);
		if (target_q[i] >= motor_lower_limit[i] && target_q[i] <= motor_higher_limit[i]){
		
			motor[i].ctrl.pos_set = target_q[i];
			delay_us(10);
		}
	}
	delay_ms(5);
}

