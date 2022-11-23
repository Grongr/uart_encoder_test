#ifndef UART_PROTOCOL_INCLUDED
#define UART_PROTOCOL_INCLUDED

#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#define COMMAND_LENGTH 8 // TODO: if it is ok - nothing

typedef enum {

	Request = 0,
	SetZero = 1,
	SetElev = 2,
	SetAzim = 3,
	ClbrADC = 4,
	ClbrEND = 5,

	CMError = -1

} UARTCommands;

void uart_init(UART_HandleTypeDef* huart);

UARTCommands read_command(char* data);

void uart_test(UART_HandleTypeDef* huart);

#endif // UART_PROTOCOL_INCLUDED
