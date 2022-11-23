#include "uart_protocol.h"

void uart_init(UART_HandleTypeDef* huart) {

	const char hello_msg[] = "Program will start in five seconds :)\r\n";
	HAL_UART_Transmit(huart, (uint8_t*)hello_msg,
			          strlen(hello_msg), HAL_MAX_DELAY);
	HAL_Delay(5000);

	const char start_msg[] = "Program started!\r\n";
	HAL_UART_Transmit(huart, (uint8_t*)start_msg,
			          strlen(start_msg), HAL_MAX_DELAY);
}

void uart_test(UART_HandleTypeDef* huart) {

	char msg[100] = "Write <Hello>\r\n";

	HAL_UART_Transmit(huart, (uint8_t*)msg,
			          strlen(msg), HAL_MAX_DELAY);

	HAL_Delay(1000);
	char getted[10] = {0};

	HAL_UART_Receive_IT(huart, (uint8_t*)getted, 5);
	uint8_t state;
	do {
		state = HAL_UART_GetState(huart);
	} while ( (state == HAL_UART_STATE_BUSY_RX) || (state == HAL_UART_STATE_BUSY_TX_RX) );

	getted[5] = '\0';

	sprintf(msg, "You've written: <%s>\r\n", getted);

	HAL_UART_Transmit(huart, (uint8_t*)msg,
			          strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(100);
}

UARTCommands read_command(char* message) {

	message[7] = '\0';

	if (message[0] == 'A')	return SetAzim;

	if (message[0] == 'E') return SetElev;

	if (!strcmp(message, "REQUEST")) return Request;

	if (!strcmp(message, "SETZERO")) return SetZero;

	if (!strcmp(message, "CLBRADC")) return ClbrADC;

	return CMError;
}
