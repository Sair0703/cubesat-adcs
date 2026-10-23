// uart.h - Register-level driver for UART0 on the LM3S6965.
#pragma once
#include <stdint.h>

void uart_init();
void uart_putc(char c);
void uart_puts(const char* s);
void uart_put_uint(uint32_t value);
