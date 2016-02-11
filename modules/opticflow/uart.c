#include <stdio.h>
#include "uart.h"
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

static int serial_handle;

int uart_open(const char *port, unsigned int baud)
{
  struct termios options;
  int i;

  serial_handle = open(port, (O_RDWR | O_NOCTTY /*| O_NDELAY*/));

  if (serial_handle < 0) {
    return errno;
  }

  /*
   * Get the current options for the port...
   */
  tcgetattr(serial_handle, &options);

  /*
   * Set the baud rates to 115200...
   */
  cfsetispeed(&options, (speed_t) baud);
  cfsetospeed(&options, (speed_t) baud);

  /*
   * Enable the receiver and set parameters ...
   */
  options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS | HUPCL);
  options.c_cflag |= (CS8 | CLOCAL | CREAD);
  options.c_lflag &= ~(ICANON | ISIG | ECHO | ECHOE | ECHOK | ECHONL | ECHOCTL | ECHOPRT | ECHOKE | IEXTEN);
  options.c_iflag &= ~(INPCK | IXON | IXOFF | IXANY | ICRNL);
  options.c_oflag &= ~(OPOST | ONLCR);

  for (i = 0; i < (int)sizeof(options.c_cc); i++) {
    options.c_cc[i] = _POSIX_VDISABLE;
  }

  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] = 1;

  /*
   * Set the new options for the port...
   */
  tcsetattr(serial_handle, TCSAFLUSH, &options);

  return (0);
}
void uart_close()
{
  close(serial_handle);
}

int uart_tx(int len, unsigned char *data)
{
  ssize_t written;

  while (len) {
    written = write(serial_handle, data, (size_t)len);
    if (written < 1) {
      return (-1);
    }
    len -= written;
    data += written;
  }

  return (0);
}

int uart_rx(int len, unsigned char *data, int timeout_ms)
{
  int l = len;
  ssize_t rread;
  struct termios options;

  tcgetattr(serial_handle, &options);
  options.c_cc[VTIME] = (cc_t)timeout_ms / 100;
  options.c_cc[VMIN] = 0;
  tcsetattr(serial_handle, TCSANOW, &options);

  while (len) {
    rread = read(serial_handle, data, (size_t)len);

    if (!rread) {
      return (0);
    } else if (rread < 0) {
      return (-1);
    }
    len -= rread;
    data += len;
  }

  return (l);
}

int uart_drain(void) {
    return (tcdrain(serial_handle));
}

