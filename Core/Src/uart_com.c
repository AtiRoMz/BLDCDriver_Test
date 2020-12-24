#include <stdio.h>
#include <string.h>
#include "main.h"

UART_HandleTypeDef *huart_com;
struct ring_buf uart_com_ring_rx;
struct ring_buf uart_com_ring_tx;


int send_lock = 0;
int previous_send_len = 0;
#define UART_COM_TIMER_MS 10
#define UART_COM_HANDLE_MS 10

void uart_com_init(UART_HandleTypeDef *huart)
{
    huart_com = huart;

    uart_com_ring_init(&uart_com_ring_rx, huart_com, RING_TYPE_RX_CIRCULAR);
    uart_com_ring_init(&uart_com_ring_tx, huart_com, RING_TYPE_TX_NORMAL);

    uart_com_handler_init();

    send_lock=0;
    previous_send_len=0;

    HAL_UART_Receive_DMA(huart_com, uart_com_ring_rx.buf, uart_com_ring_rx.buf_size);
    HAL_Delay(10);
}

static void uart_com_putc(uint8_t c){
    uart_com_ring_putc(&uart_com_ring_tx, c);
    if(!send_lock){
        uint16_t len = (uint16_t)uart_com_ring_available_linear(&uart_com_ring_tx);
        previous_send_len = len;
        send_lock = 1;
       if(len != 0){
        HAL_UART_Transmit_DMA(uart_com_ring_tx.huart, uart_com_ring_tx.buf+uart_com_ring_get_r_ptr(&uart_com_ring_tx), len);
       }
    }
}

void uart_com_send(uint8_t tag, handler_arg value)
{

    uart_com_putc(UART_COM_START_0);
    uart_com_putc(UART_COM_START_1);
    uart_com_putc(tag);
    uart_com_putc((value.u32_val>>24)&0xFF);
    uart_com_putc((value.u32_val>>16)&0xFF);
    uart_com_putc((value.u32_val>>8)&0xFF);
    uart_com_putc(value.u32_val&0xFF);
    uart_com_putc(UART_COM_END_0);
    uart_com_putc(UART_COM_END_1);
}

void uart_com_send_it(UART_HandleTypeDef *huart) {
    if(uart_com_ring_tx.huart != huart) return;

    uart_com_ring_forward_r_ptr(&uart_com_ring_tx, previous_send_len);
    uint16_t len = (uint16_t)uart_com_ring_available_linear(&uart_com_ring_tx);
    if (len > 0 ) {
        previous_send_len = len;
        send_lock = 1;
        HAL_UART_Transmit_DMA(uart_com_ring_tx.huart, uart_com_ring_tx.buf+uart_com_ring_get_r_ptr(&uart_com_ring_tx), len);
    }else{
        send_lock = 0;
    }
}

void uart_com_proc(){
    static enum uart_com_recv_st st = UART_COM_RECV_ST_WAIT_START_0;
    static uint8_t tag;
    static handler_arg value;
    uint8_t tmp[1];

    if(uart_com_ring_available(&uart_com_ring_rx) == 0) return;
    while(uart_com_ring_available(&uart_com_ring_rx) > 0){
        uart_com_ring_getc(&uart_com_ring_rx, tmp);
        switch(st){
            case UART_COM_RECV_ST_WAIT_START_0:
                if(tmp[0] == UART_COM_START_0) {
                    st = UART_COM_RECV_ST_WAIT_START_1;
                }else{
                    st = UART_COM_RECV_ST_WAIT_START_0;
                }
                break;
            case UART_COM_RECV_ST_WAIT_START_1:
                if(tmp[0] == UART_COM_START_1){
                    st = UART_COM_RECV_ST_WAIT_TAG;
                }else{
                    st = UART_COM_RECV_ST_WAIT_START_0;
                }
                break;
            case UART_COM_RECV_ST_WAIT_TAG:
                tag = tmp[0];
                value.u32_val = 0;
                st = UART_COM_RECV_ST_WAIT_DATA_0;
                break;
            case UART_COM_RECV_ST_WAIT_DATA_0:
                value.u32_val = tmp[0]<<24;
                st = UART_COM_RECV_ST_WAIT_DATA_1;
                break;
            case UART_COM_RECV_ST_WAIT_DATA_1:
                value.u32_val += tmp[0]<<16;
                st = UART_COM_RECV_ST_WAIT_DATA_2;
                break;
            case UART_COM_RECV_ST_WAIT_DATA_2:
                value.u32_val += tmp[0]<<8;
                st = UART_COM_RECV_ST_WAIT_DATA_3;
                break;
            case UART_COM_RECV_ST_WAIT_DATA_3:
                value.u32_val += tmp[0];
                st = UART_COM_RECV_ST_WAIT_END_0;
                break;
            case UART_COM_RECV_ST_WAIT_END_0:
                if(tmp[0] == UART_COM_END_0) {
                    st = UART_COM_RECV_ST_WAIT_END_1;
                }else{
                    st = UART_COM_RECV_ST_WAIT_START_0;
                }
                break;
            case UART_COM_RECV_ST_WAIT_END_1:
                if(tmp[0] == UART_COM_END_1){
                    uart_com_handler_handle(tag, value);
                }
                st = UART_COM_RECV_ST_WAIT_START_0;
                break;
            default:
                st = UART_COM_RECV_ST_WAIT_START_0;
                break;
        }
    }
}

void uart_com_timer(){
    uart_com_proc();
}
