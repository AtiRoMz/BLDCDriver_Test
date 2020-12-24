//
// Created by naoki on 19/03/03.
//
#include "main.h"
#include <stdio.h>
uart_com_handler com_handler[UART_COM_MAX_HANDLER];



/*
void l_chika(handler_arg value){
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

void echo_back(handler_arg value){
    handler_arg tx_data;
    tx_data.u32_val = value.u32_val+1;
    printf("value:%d\n", (int)tx_data.u32_val);
    uart_com_send(UART_COM_HANDLER_PRINT_U32, tx_data);
}

void print_u32(handler_arg value){
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}
*/


void uart_com_handler_init(){

    //Add recv handler
//    com_handler[UART_COM_HANDLER_L_CHIKA] = l_chika;
//    com_handler[UART_COM_HANDLER_ECHO_BACK] = echo_back;
//    com_handler[UART_COM_HANDLER_PRINT_U32] = print_u32;
}

void uart_com_handler_handle(uint8_t tag, handler_arg value){
    uart_com_handler handler = com_handler[tag];
    handler(value);
}
