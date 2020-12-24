#include "main.h"
#include <stdio.h>

static void update_w_ptr(struct ring_buf *ring){
    if(ring->type == RING_TYPE_RX_CIRCULAR){
        ring->w_ptr = (uint16_t) ((ring->buf_size - ring->huart->hdmarx->Instance->W_PTR) & 0xFFFF);
    }
}


void uart_com_ring_init(struct ring_buf *ring, UART_HandleTypeDef *huart, int type){
  ring->buf_size = RING_BUF_SIZE_UART;
  ring->w_ptr = 0;
  ring->r_ptr = 0;
  ring->overwrite_cnt = 0;
  ring->type = type;
  ring->huart = huart;
}

int uart_com_ring_getc(struct ring_buf *ring, uint8_t *c){
    update_w_ptr(ring);

    if(ring->r_ptr == ring->w_ptr) return RING_FAIL;
    uint16_t next_ptr = (uint16_t)(ring->r_ptr+1);
    if(next_ptr >= ring->buf_size) next_ptr = 0;

    *c = ring->buf[ring->r_ptr];
    ring->r_ptr = next_ptr;
    return RING_SUCCESS;
}

int uart_com_ring_putc(struct ring_buf *ring, uint8_t c){
    update_w_ptr(ring);

    uint16_t next_ptr = (uint16_t)(ring->w_ptr+1);

    if(next_ptr >= ring->buf_size) next_ptr = 0;

    if(next_ptr == ring->r_ptr){
        ring->overwrite_cnt++;
        return RING_FAIL;
    }
    ring->buf[ring->w_ptr] = c;
    ring->w_ptr = next_ptr;
    return RING_SUCCESS;
}

int uart_com_ring_available(struct ring_buf *ring){
    update_w_ptr(ring);

    if(ring->w_ptr >= ring->r_ptr){
        return ring->w_ptr - ring->r_ptr;
    }else{
        return ring->buf_size + ring->w_ptr - ring->r_ptr;
    }
}

int uart_com_ring_available_linear(struct ring_buf *ring){
    update_w_ptr(ring);

    if(ring->w_ptr >= ring->r_ptr){
        return ring->w_ptr - ring->r_ptr;
    }else{
        return ring->buf_size - ring->r_ptr;
    }
}

uint16_t uart_com_ring_get_w_ptr(struct ring_buf *ring){
    update_w_ptr(ring);

    return ring->w_ptr;
}

uint16_t uart_com_ring_get_r_ptr(struct ring_buf *ring){
    update_w_ptr(ring);

    return ring->r_ptr;
}

void uart_com_ring_forward_r_ptr(struct ring_buf *ring, int len){
    update_w_ptr(ring);

    while(len > 0){
        if(ring->r_ptr+1 >= ring->buf_size){
            ring->r_ptr = 0;
        }else{
            ring->r_ptr += 1;
        }
        len--;
    }
}

void uart_com_ring_set_w_ptr(struct ring_buf *ring, uint16_t w_ptr){
    ring->w_ptr = w_ptr;
}

void uart_com_ring_debug(struct ring_buf *ring){
    update_w_ptr(ring);
    printf("\n====Ring Debug information====\n");
    printf("Buffer Size: %d\n", ring->buf_size);
    printf("Write Pointer: %d\n", ring->w_ptr);
    printf("Read Pointer: %d\n", ring->r_ptr);
    printf("OverWrite Count: %d\n\n", ring->overwrite_cnt);
}
