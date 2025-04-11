#ifndef __PRINT_H
#define __PRINT_H
#include "main.h"
#include <stdio.h>

extern UART_HandleTypeDef huart2;
#define Print_UART huart2

int fputc(int ch, FILE *f);
int fgetc(FILE *f);
#endif
