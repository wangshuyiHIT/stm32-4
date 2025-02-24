#include <string.h>
#include "usbsent.h"
#include "dm4310_drv.h"
#include "dm4310_ctrl.h"
#include "usbd_cdc_if.h"

#define DATASIZE 50
#define StartByte 0xAB
#define EndByte 0xBA

uint8_t buffer[DATASIZE];

void send_data(motor_t *motor){
	//ÉèÖÃÖ¡Í·
	buffer[0] = StartByte;
	//ÉèÖÃÖ¡Î²
	buffer[DATASIZE - 1] = EndByte;
	//±àÂëÐ´Èëdata
	buffer[1] = (motor[Motor1].para.p_int >> 8);
	buffer[2] = motor[Motor1].para.p_int;
	buffer[3] = (motor[Motor2].para.p_int >> 8);
	buffer[4] = motor[Motor2].para.p_int;
	buffer[5] = (motor[Motor3].para.p_int >> 8);
	buffer[6] = motor[Motor3].para.p_int;
	buffer[7] = (motor[Motor4].para.p_int >> 8);
	buffer[8] = motor[Motor4].para.p_int;
	
	buffer[9] = (motor[Motor5].para.p_int >> 8);
	buffer[10] = motor[Motor5].para.p_int;
	buffer[11] = (motor[Motor6].para.p_int >> 8);
	buffer[12] = motor[Motor6].para.p_int;
	buffer[13] = (motor[Motor7].para.p_int >> 8);
	buffer[14] = motor[Motor7].para.p_int;
	buffer[15] = (motor[Motor8].para.p_int >> 8);
	buffer[16] = motor[Motor8].para.p_int;
	
	buffer[17] = (motor[Motor9].para.p_int >> 8);
	buffer[18] = motor[Motor9].para.p_int;
	buffer[19] = (motor[MotorA].para.p_int >> 8);
	buffer[20] = motor[MotorA].para.p_int;
	buffer[21] = (motor[MotorB].para.p_int >> 8);
	buffer[22] = motor[MotorB].para.p_int;
	buffer[23] = (motor[MotorC].para.p_int >> 8);
	buffer[24] = motor[MotorC].para.p_int;


	buffer[25] = (motor[Motor1].para.v_int >> 8);
	buffer[26] = motor[Motor1].para.v_int;
	buffer[27] = (motor[Motor2].para.v_int >> 8);
	buffer[28] = motor[Motor2].para.v_int;
	buffer[29] = (motor[Motor3].para.v_int >> 8);
	buffer[30] = motor[Motor3].para.v_int;
	buffer[31] = (motor[Motor4].para.v_int >> 8);
	buffer[32] = motor[Motor4].para.v_int;
	
	buffer[33] = (motor[Motor5].para.v_int >> 8);
	buffer[34] = motor[Motor5].para.v_int;
	buffer[35] = (motor[Motor6].para.v_int >> 8);
	buffer[36] = motor[Motor6].para.v_int;
	buffer[37] = (motor[Motor7].para.v_int >> 8);
	buffer[38] = motor[Motor7].para.v_int;
	buffer[39] = (motor[Motor8].para.v_int >> 8);
	buffer[40] = motor[Motor8].para.v_int;
	
	buffer[41] = (motor[Motor9].para.v_int >> 8);
	buffer[42] = motor[Motor9].para.v_int;
	buffer[43] = (motor[MotorA].para.v_int >> 8);
	buffer[44] = motor[MotorA].para.v_int;
	buffer[45] = (motor[MotorB].para.v_int >> 8);
	buffer[46] = motor[MotorB].para.v_int;
	buffer[47] = (motor[MotorC].para.v_int >> 8);
	buffer[48] = motor[MotorC].para.v_int;
	
	CDC_Transmit_HS(buffer,DATASIZE);
}
