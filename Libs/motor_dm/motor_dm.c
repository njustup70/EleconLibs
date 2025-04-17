#include "motor_dm.h"

/**
 * @name 获得位置速度数组
 * @details 输入位置和速度（均为RAD），获得一个可以发送给达妙电机的数组
 */
void DM_get_posispd_buffer(uint8_t buf[], float posi, float speed)
{
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&posi;
	vbuf=(uint8_t*)&speed;

    buf[0] = *pbuf;
    buf[1] = *(pbuf+1);
    buf[2] = *(pbuf+2);
    buf[3] = *(pbuf+3);
    
    buf[4] = *vbuf;
    buf[5] = *(vbuf+1);
    buf[6] = *(vbuf+2);
    buf[7] = *(vbuf+3);
}



/**
 * @name 给达妙发送消息
 * @details 默认CAN1
 */
void DM_buf_send(uint8_t id, uint8_t bufs[], int id_offset)
{
  static uint32_t txmailbox;		        // CAN 邮箱
  CAN_TxHeaderTypeDef TxMsg;		        // TX 消息

  TxMsg.DLC = (uint8_t)8 ;		          // CAN 消息长度设置为8个字节
  TxMsg.RTR = CAN_RTR_DATA ;		        // 非远程帧，普通数据帧
  TxMsg.IDE = CAN_ID_STD ;		          // 发送的帧是扩展帧
  TxMsg.StdId = id_offset + id;             // CAN_ID
  TxMsg.TransmitGlobalTime = DISABLE; 

  // 发送消息
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);     // 等待CAN邮箱

	HAL_CAN_AddTxMessage(&hcan1, &TxMsg, bufs, &txmailbox);
}

/**
 * @name 给达妙发送消息
 * @details 默认CAN1
 */
void DM_buf_send_can(CAN_HandleTypeDef* hcan, uint8_t id, uint8_t bufs[], int id_offset)
{
  static uint32_t txmailbox;		        // CAN 邮箱
  CAN_TxHeaderTypeDef TxMsg;		        // TX 消息

  TxMsg.DLC = (uint8_t)8 ;		          // CAN 消息长度设置为8个字节
  TxMsg.RTR = CAN_RTR_DATA ;		        // 非远程帧，普通数据帧
  TxMsg.IDE = CAN_ID_STD ;		          // 发送的帧是扩展帧
  TxMsg.StdId = id_offset + id;             // CAN_ID
  TxMsg.TransmitGlobalTime = DISABLE; 

  // 发送消息
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);     // 等待CAN邮箱

	HAL_CAN_AddTxMessage(&hcan, &TxMsg, bufs, &txmailbox);
}