#ifndef UART_H
#define UART_H

int uart_open(const char *port, unsigned int speed);
void uart_close(void);
int uart_tx(int len, unsigned char *data);
int uart_rx(int len, unsigned char *data, int timeout_ms);
int uart_drain(void);

#endif // UART_H
