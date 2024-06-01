/*******************************************************************************************************
**                                           File description
** File Name:           dev_uart.h
** Description:         南向串口通信接口定义
** Creator:             jianjun_xia
** Creation Date:       2024年5月29日
** Modify Log:          none
** Last Modified Date:  2023年5月29日
*******************************************************************************************************/
#ifndef __DEV_UART_H_
#define __DEV_UART_H_

/* If this is a C++ compiler, use C linkage */
#if defined(__cplusplus)
extern "C"
{
#endif

#include <stdio.h>

#define DEV_PORT "/dev/ttyS2"  ///< 网关可用串口设备文件路径


///< @brief     初始化设备串口
///< @param     dev_port  [in]          设备串口路径
///< @param     baudRate  [in]          串口波特率
///< @param     databits  [in]          数据位   取值为 7 或者8
///< @param     parity    [in]          效验类型 取值为N,E,O,,S
///< @param     stopbits  [in]          停止位   取值为 1 或者2
///< @param     nonblock  [in]          是否阻塞
///< @param     fd        [out]         串口文件描述符
///< @return    -1  打开失败,            0   成功
int dev_uart_init(const char *dev_port, const char *baudRate, const char dataBits, const char parity, const char stopBits, int nonblock, int *fd);


///< @brief     取消初始化设备串口
///< @param     fd  [in]          串口文件描述符
///< @return    -1  打开失败,      0   成功
int dev_uart_deinit(int fd);


///< @brief    串口发送数据
///< @param    fd        [in]       串口文件描述符
///< @param    send_buf  [in]       发送数据
///< @param    buf_len  [in]        数据的个数
///< @return   -1 配置失败,          大于0 实际发送的数据个数
int dev_uart_send(int fd, char *send_buf, int buf_len);


///< @brief    串口接收数据
///< @param    fd        [in]        串口文件描述符
///< @param    recv_buf  [out]       接收数据
///< @param    buf_len   [out]       数据的个数
///< @return   -1 配置失败,           大于0 实际接收的数据个数
int dev_uart_recv(int fd, char *recv_buf, int buf_len);


/* If this is a C++ compiler, use C linkage */
#if defined(__cplusplus)
}
#endif

#endif
/*******************************************************************************************************
**                                           End of file
*******************************************************************************************************/
