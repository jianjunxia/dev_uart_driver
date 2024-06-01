/*******************************************************************************************************
**                                           File description
** File Name:           dev_uart.c
** Description:         南向串口通信接口实现
** Creator:             jianjun_xia
** Creation Date:       2024年5月29日
** Modify Log:          none
** Last Modified Date:  2023年5月29日
*******************************************************************************************************/
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <termios.h>
#include <linux/serial.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include "dev_uart.h"

static int speed_arr[] = {  B50,      B75,      B110,      B134,      B150,      B200,       B300,       B600,      B1200,      B1800,
                            B2400,    B4800,    B9600,     B19200,    B38400,    B57600,     B115200,    B230400,   B460800,    B500000,
                            B576000,  B921600,  B1000000,  B1152000,  B1500000,  B2000000,   B2500000,   B3000000,  B3500000,   B4000000
                         };

static const char *name_arr[] = { "50",      "75",       "110",    "134",      "150",      "200",      "300",       "600",     "1200",     "1800", 
                           "2400",   "4800",     "9600",    "19200",    "38400",    "57600",    "115200",   "230400",   "460800",   "500000",
                           "576000", "921600", "1000000",   "1152000",  "1500000",  "2000000",  "2500000",  "3000000",  "3500000",  "4000000"
                         };


static speed_t dev_uart_get_baudrate(const char *baudRate)
{   
    if (baudRate == NULL) {
        printf("%s, invalid parameter!\n", __func__);
        return 0;
    }

    size_t i;
    for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
        
        if (strcmp(baudRate, name_arr[i]) == 0) {
            printf("%s, baudRate %s", __func__, name_arr[i]);
            return speed_arr[i];
        }
    }

    return 0;
}

/// @brief 串口配置函数
/// @param fd 
/// @param baudRate 
/// @param dataBits 
/// @param parity 
/// @param stopBits 
/// @return -1 fail, 0 success
static int dev_uart_config(int fd, const char *baudRate, const char dataBits, const char parity, const char stopBits)
{
    if (fd <= 0 || baudRate == NULL) { 
        printf("%s, invalid parameter!\n", __func__);
        return -1;
    }

    speed_t speed;
    struct termios options;

    ///< tcgetattr(fd,&options)得到与fd指向对象的相关参数，
    ///< 并将它们保存于options, 该函数还可以测试配置是否正确, 
    ///< 该串口是否可用等, 若调用成功, 函数返回值为0，若调用失败，函数返回值为1
    if ( tcgetattr(fd, &options) != 0) {
        printf("%s, uart_set tcgetattr init fail.", __func__);
        return -1;
    }

    ///< 修改控制模式，保证程序不会占用串口
    ///< 修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= (CLOCAL | CREAD);

    ///< 设置数据位
    switch (dataBits) {
        case '7':
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS7;
            break;
        case '8':
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS8;
            break;
        default:
            printf("%s, uart_set Unsupported databits.", __func__);
            return -1;
    }

    ///< 设置效验类型
    switch (parity) {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O':
            options.c_cflag |= PARENB;
            options.c_cflag |= PARODD;
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's':
        case 'S':
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            options.c_iflag |= INPCK;
            break;
        default:
            printf("%s, uart_set Unsupported parity.", __func__);
            return -1;
    }

    ///< 设置停止位
    switch (stopBits) {
        case '1':
            options.c_cflag &= ~CSTOPB;
            break;
        case '2':
            options.c_cflag |= CSTOPB;
            break;
        default:
            printf("%s, uart_set Unsupported stopbits.", __func__);
            return -1;
    }

    ///< 设置波特率
    speed = dev_uart_get_baudrate(baudRate);
    if (speed == 0) {
        printf("%s, uart_set Invalid baudrate.", __func__);
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    options.c_oflag &= ~OPOST;
    options.c_oflag &= ~(ONLCR | OCRNL);

    options.c_iflag &= ~(ICRNL | INLCR);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    tcflush(fd, TCIFLUSH);

    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        printf("%s, uart_set tcgetattr TCSANOW fail.", __func__);
        return -1;
    }

    return 0;
}


int dev_uart_init(const char *dev_port, const char *baudRate, const char dataBits, const char parity, const char stopBits, int nonblock, int *fd)
{
    if (dev_port == NULL || baudRate == NULL ||  nonblock < 0 || fd == NULL) { 
        printf("%s, invalid parameter!\n", __func__);
        return -1;
    }

    int open_flags = O_RDWR | O_NOCTTY;
    
    if (nonblock) {
        open_flags |= O_NONBLOCK;
    }

    *fd = open(dev_port, open_flags);
    if (*fd < 0) {
        printf("%s, open %s fail!\n", __func__, dev_port);
        return -1;
    }

    if (dev_uart_config(*fd, baudRate, dataBits, parity, stopBits) != 0) {
        printf("%s, uart set param error, errno %d\n", __func__, errno);
        if (*fd > 0) {
            close(*fd);
        }
        return -1;
    }

    tcflush(*fd, TCIOFLUSH);

    return 0;
}


int dev_uart_deinit(int fd)
{
    if (fd <= 0) {
        printf("%s, invalid parameter!\n", __func__);
        return -1;
    }
    close(fd);

    return 0;
}


int dev_uart_send(int fd, char *send_buf, int buf_len)
{
    if (fd <= 0 || send_buf == NULL || buf_len <= 0) {
        printf("%s, invalid parameter!\n", __func__);
        return -1;
    }
    
    int total_written = 0;
    int write_len = 0;

    while (buf_len) {
        write_len = write(fd, send_buf, buf_len);
        if (write_len == -1) {
            if (EAGAIN == errno) {
                usleep(10000); // 10 ms
                continue;
            } else {
                tcflush(fd, TCOFLUSH);
                return -1;
            }
        } else {
            buf_len -= write_len;
            send_buf += write_len;
            total_written += write_len;
        }
    }

    return total_written;
}


int dev_uart_recv(int fd, char *recv_buf, int buf_len)
{
    if (fd <= 0 || recv_buf == NULL || buf_len <= 0) {
        printf("%s, invalid parameter!\n", __func__);
        return -1;
    }

    fd_set fs_read;
    int len = 0;
    int ret = 0;
    int timeout = 1;
    int read_error_try = 2;

    struct timeval time;
    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);

    ///< 超时时间1s
    time.tv_sec  = timeout;  
    time.tv_usec = 0;

    int fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
    if (fs_sel == -1) {
        printf("%s, select error.\n", __func__);
        return -1;
    } else if (fs_sel == 0) {
        return -1; // Timeout
    }else{
        if (FD_ISSET(fd, &fs_read)) {
            while (1) {
                ret = read(fd, recv_buf + len, buf_len - len);
                if (ret > 0) {
                    len += ret;
                } else {
                    if (errno == EAGAIN) {
                        usleep(10000);
                        continue;
                    } else if (--read_error_try == 0) {
                        break;
                    }
                }
            }
            
            return len;
        }
    }

    return -1;
}

/*******************************************************************************************************
**                                           End of file
*******************************************************************************************************/
