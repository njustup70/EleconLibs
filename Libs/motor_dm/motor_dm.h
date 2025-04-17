#ifndef MOTOR_DM_H
#define MOTOR_DM_H
#include "stm32f4xx_hal.h"
#include "can.h"

#define DM_HALFROUND_PERSEC 1.57
#define DM_ROUND_PERSEC 3.14
#define DM_TWOROUND_PERSEC 6.28

void DM_get_posispd_buffer(uint8_t buf[], float posi, float speed);
void DM_buf_send(uint8_t id, uint8_t bufs[], int id_offset);
void DM_buf_send_can(CAN_HandleTypeDef* hcan, uint8_t id, uint8_t bufs[], int id_offset);

#endif