#include "dm4310_drv.h"
#include "dm4310_ctrl.h"
#include "string.h"

motor_t motor[num];

/**
************************************************************************
* @brief:      	dm4310_motor_init: DM4310�����ʼ������
* @param:      	void
* @retval:     	void
* @details:    	��ʼ������DM4310�ͺŵĵ��������Ĭ�ϲ����Ϳ���ģʽ��
*               �ֱ��ʼ��Motor1��Motor2������ID������ģʽ������ģʽ����Ϣ��
************************************************************************
**/
void dm4310_motor_init(void)
{
	// ��ʼ��Motor1��Motor2�ĵ���ṹ
	memset(&motor[Motor1], 0, sizeof(motor[Motor1]));
	memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
	memset(&motor[Motor3], 0, sizeof(motor[Motor3]));
	memset(&motor[Motor4], 0, sizeof(motor[Motor4]));
	memset(&motor[Motor5], 0, sizeof(motor[Motor5]));
	memset(&motor[Motor6], 0, sizeof(motor[Motor6]));
	
	memset(&motor[Motor7], 0, sizeof(motor[Motor7]));
	memset(&motor[Motor8], 0, sizeof(motor[Motor8]));
	memset(&motor[Motor9], 0, sizeof(motor[Motor9]));
	memset(&motor[MotorA], 0, sizeof(motor[MotorA]));
	memset(&motor[MotorB], 0, sizeof(motor[MotorB]));
	memset(&motor[MotorC], 0, sizeof(motor[MotorC]));
	// ����Motor1�ĵ����Ϣ
	motor[Motor1].id = 0x01;
	motor[Motor1].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
	motor[Motor1].ctrl.vel_set = 0.0f;
	motor[Motor1].ctrl.kd_set = 2.0f;
	
	motor[Motor2].id = 0x02;
	motor[Motor2].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
	motor[Motor2].ctrl.vel_set = 0.0f;
	motor[Motor2].ctrl.kd_set = 2.0f;
	
	motor[Motor3].id = 0x03;
	motor[Motor3].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
	motor[Motor3].ctrl.vel_set = 0.0f;
	motor[Motor3].ctrl.kd_set = 2.0f;
	
	motor[Motor4].id = 0x04;
	motor[Motor4].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
	motor[Motor4].ctrl.vel_set = 0.0f;
	motor[Motor4].ctrl.kd_set = 2.0f;
	
	motor[Motor5].id = 0x05;
	motor[Motor5].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
	motor[Motor5].ctrl.vel_set = 0.0f;
	motor[Motor5].ctrl.kd_set = 2.0f;
	
	motor[Motor6].id = 0x06;
	motor[Motor6].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
	motor[Motor6].ctrl.vel_set = 0.0f;
	motor[Motor6].ctrl.kd_set = 2.0f;
	
	
	motor[Motor7].id = 0x07;
	motor[Motor7].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
	motor[Motor7].ctrl.vel_set = 0.0f;
	motor[Motor7].ctrl.kd_set = 2.0f;
	
	motor[Motor8].id = 0x08;
	motor[Motor8].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
	motor[Motor8].ctrl.vel_set = 0.0f;
	motor[Motor8].ctrl.kd_set = 2.0f;
	
	motor[Motor9].id = 0x09;
	motor[Motor9].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
	motor[Motor9].ctrl.vel_set = 0.0f;
	motor[Motor9].ctrl.kd_set = 2.0f;
	
	motor[MotorA].id = 0x0A;
	motor[MotorA].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
	motor[MotorA].ctrl.vel_set = 0.0f;
	motor[MotorA].ctrl.kd_set = 2.0f;
	
	motor[MotorB].id = 0x0B;
	motor[MotorB].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
	motor[MotorB].ctrl.vel_set = 0.0f;
	motor[MotorB].ctrl.kd_set = 2.0f;
	
	motor[MotorC].id = 0x0C;
	motor[MotorC].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
	motor[MotorC].ctrl.vel_set = 0.0f;
	motor[MotorC].ctrl.kd_set = 2.0f;
}

/**
************************************************************************
* @brief:      	fdcan1_rx_callback: CAN1���ջص�����
* @param:      	void
* @retval:     	void
* @details:    	����CAN1�����жϻص������ݽ��յ���ID�����ݣ�ִ����Ӧ�Ĵ���
*               �����յ�IDΪ0ʱ������dm4310_fbdata��������Motor�ķ������ݡ�
************************************************************************
**/
void fdcan1_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	fdcanx_receive(&hfdcan1, &rec_id, rx_data);
	switch (rec_id)
	{
 		case 0x11: dm4310_fbdata(&motor[Motor1], rx_data); break;
		case 0x12: dm4310_fbdata(&motor[Motor2], rx_data); break;
		case 0x13: dm4310_fbdata(&motor[Motor3], rx_data); break;
		case 0x14: dm4310_fbdata(&motor[Motor4], rx_data); break;
		case 0x15: dm4310_fbdata(&motor[Motor5], rx_data); break;
		case 0x16: dm4310_fbdata(&motor[Motor6], rx_data); break;
	}
}

void fdcan2_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	fdcanx_receive(&hfdcan2, &rec_id, rx_data);
	switch (rec_id)
	{
 		case 0x17: dm4310_fbdata(&motor[Motor7], rx_data); break;
		case 0x18: dm4310_fbdata(&motor[Motor8], rx_data); break;
		case 0x19: dm4310_fbdata(&motor[Motor9], rx_data); break;
		case 0x1A: dm4310_fbdata(&motor[MotorA], rx_data); break;
		case 0x1B: dm4310_fbdata(&motor[MotorB], rx_data); break;
		case 0x1C: dm4310_fbdata(&motor[MotorC], rx_data); break;
	}
}


