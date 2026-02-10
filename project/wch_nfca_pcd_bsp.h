/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_pcd_bsp.h
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2024/11/14
 * Description        : NFC-A PCD BSP底层接口
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef _WCH_NFCA_PCD_BSP_H_
#define _WCH_NFCA_PCD_BSP_H_
#define  BSP_VERSION "V1.1"
#include "wch_nfca_pcd_config.h"
#include "CH58x_NFCA_LIB.h"

#define PU8_BUF(BUF)                    ((uint8_t *)BUF)
#define PU16_BUF(BUF)                   ((uint16_t *)BUF)
#define PU32_BUF(BUF)                   ((uint32_t *)BUF)

#define NFCA_PCD_MAX_SEND_NUM           (NFCA_PCD_DATA_BUF_SIZE)
#define NFCA_PCD_MAX_RECV_NUM           (NFCA_PCD_DATA_BUF_SIZE * 16 / 9)
#define NFCA_PCD_MAX_PARITY_NUM         (NFCA_PCD_MAX_RECV_NUM)

extern uint8_t g_nfca_pcd_send_buf[((NFCA_PCD_MAX_SEND_NUM + 3) & 0xfffc)];
extern uint8_t g_nfca_pcd_recv_buf[((NFCA_PCD_MAX_RECV_NUM + 3) & 0xfffc)];
extern uint8_t g_nfca_pcd_parity_buf[NFCA_PCD_MAX_PARITY_NUM];

extern uint16_t g_nfca_pcd_recv_buf_len;
extern uint32_t g_nfca_pcd_recv_bits;

/*******************************************************************************
 * @fn              nfca_pcd_init
 *
 * @brief           nfc-a pcd初始�?
 *
 * @param           None.
 *
 * @return          None.
 */
extern void nfca_pcd_init(void);

/*******************************************************************************
 * @fn              nfca_pcd_start
 *
 * @brief           nfc-a pcd开始运行，开始发送连续波
 *
 * @param           None.
 *
 * @return          None.
 */
extern void nfca_pcd_start(void);

/*******************************************************************************
 * @fn              nfca_pcd_stop
 *
 * @brief           nfc-a pcd停止运行，停止发送连续波
 *
 * @param           None.
 *
 * @return          None.
 */
extern void nfca_pcd_stop(void);

/*******************************************************************************
 * @fn              nfca_pcd_lpcd_calibration
 *
 * @brief           nfc-a pcd lpcd ADC检卡方式阈值校�?
 *
 * @param           None.
 *
 * @return          None.
 */
extern void nfca_pcd_lpcd_calibration(void);

/*******************************************************************************
 * @fn              nfca_pcd_lpcd_check
 *
 * @brief           nfc-a pcd lpcd ADC检�?
 *
 * @param           None.
 *
 * @return          1 有卡�?0无卡.
 */
extern uint8_t nfca_pcd_lpcd_check(void);

/*******************************************************************************
 * @fn              nfca_adc_get_ant_signal
 *
 * @brief           nfc-a pcd lpcd ADC检�?
 *
 * @param           None.
 *
 * @return          uint16_t，返回天线上信号的adc检测�?.
 */
extern uint16_t nfca_adc_get_ant_signal(void);

/*******************************************************************************
 * @fn              nfca_pcd_wait_communicate_end
 *
 * @brief           nfc-a pcd 等待通信结束
 *
 * @param           None.
 *
 * @return          nfca_pcd_controller_state_t，返回通讯结束状�?.
 */
extern nfca_pcd_controller_state_t nfca_pcd_wait_communicate_end(void);

/*******************************************************************************
 * @fn              nfca_pcd_rand
 *
 * @brief           nfc-a pcd 获取随机数接�?
 *
 * @param           None.
 *
 * @return          uint32_t，返回一个随机数
 */
extern uint32_t nfca_pcd_rand(void);

/*******************************************************************************
 * @fn              nfca_pcd_ctr_init
 *
 * @brief           nfc-a pcd 天线信号控制初始�?
 *
 * @param           None.
 *
 * @return          None.
 */
extern void nfca_pcd_ctr_init(void);

/*******************************************************************************
 * @fn              nfca_pcd_ctr_handle
 *
 * @brief           nfc-a pcd 天线信号控制处理
 *
 * @param           None.
 *
 * @return          None.
 */
extern void nfca_pcd_ctr_handle(void);

#endif /* _WCH_NFCA_PCD_BSP_H_ */
