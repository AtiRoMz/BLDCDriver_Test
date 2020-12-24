//
// Created by naoki on 19/02/23.
//


#ifndef STM32_UART_STATIC_UART_COM_H
#define STM32_UART_STATIC_UART_COM_H
#include "main.h"

#define UART_COM_START_0 0xFF
#define UART_COM_START_1 0xFE

#define UART_COM_END_0 0x00
#define UART_COM_END_1 0x01

#define UART_COM_MAX_HANDLER 256

#define F7

#ifdef F7
#define W_PTR NDTR
#endif

#ifdef F3
#define W_PTR CNDTR
#endif
typedef union {
    uint32_t u32_val;
    float float_val;
} handler_arg;

void uart_com_send(uint8_t tag, handler_arg value);
void uart_com_send_it(UART_HandleTypeDef *huart);
void uart_com_init(UART_HandleTypeDef *huart);
void uart_com_timer();

enum uart_com_recv_st {
    UART_COM_RECV_ST_WAIT_START_0,
    UART_COM_RECV_ST_WAIT_START_1,
    UART_COM_RECV_ST_WAIT_TAG,
    UART_COM_RECV_ST_WAIT_DATA_0,
    UART_COM_RECV_ST_WAIT_DATA_1,
    UART_COM_RECV_ST_WAIT_DATA_2,
    UART_COM_RECV_ST_WAIT_DATA_3,
    UART_COM_RECV_ST_WAIT_END_0,
    UART_COM_RECV_ST_WAIT_END_1,
};

/*
enum uart_com_handler_tag {
    UART_COM_HANDLER_L_CHIKA = 0,
    UART_COM_HANDLER_ECHO_BACK = 1,
    UART_COM_HANDLER_PRINT_U32 = 2,

};
*/

#define RING_SUCCESS 0
#define RING_FAIL 1
#define RING_BUF_SIZE_UART 128

#define RING_TYPE_RX_NORMAL 0
#define RING_TYPE_RX_CIRCULAR 1
#define RING_TYPE_TX_NORMAL 2
struct ring_buf {
    uint8_t buf[RING_BUF_SIZE_UART];
    uint16_t buf_size;
    uint16_t w_ptr, r_ptr;
    uint16_t overwrite_cnt;
    int type;
    UART_HandleTypeDef *huart;
};

void uart_com_ring_init(struct ring_buf *ring, UART_HandleTypeDef *huart, int type);
int uart_com_ring_getc(struct ring_buf *ring, uint8_t *c);
int uart_com_ring_putc(struct ring_buf *ring, uint8_t c);
int uart_com_ring_available(struct ring_buf *ring);
int uart_com_ring_available_linear(struct ring_buf *ring);
uint16_t uart_com_ring_get_w_ptr(struct ring_buf *ring);
uint16_t uart_com_ring_get_r_ptr(struct ring_buf *ring);
void uart_com_ring_forward_r_ptr(struct ring_buf *ring, int len);
void uart_com_ring_set_w_ptr(struct ring_buf *ring, uint16_t w_ptr);
void uart_com_ring_debug(struct ring_buf *ring);
void uart_com_proc();
typedef void (* uart_com_handler)(handler_arg arg);

void uart_com_handler_init();
void uart_com_handler_handle(uint8_t tag, handler_arg value);


#endif //STM32_UART_STATIC_UART_COM_H
