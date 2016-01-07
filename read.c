#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#define SERIAL_PORT "/dev/usbmon0"

/*
   Code by: v4char
   Speed: 9600 buad
   email: v4char@gmail.com
*/

int main()
{

   int fd;
   fd = open(SERIAL_PORT ,O_RDWR | O_NOCTTY);    
   struct termios SerialPortSettings;
   tcgetattr(fd, &SerialPortSettings);
   cfsetispeed(&SerialPortSettings,B9600);
   cfsetospeed(&SerialPortSettings,B9600);

   SerialPortSettings.c_cflag &= ~PARENB;
   SerialPortSettings.c_cflag &= ~CSTOPB;
   SerialPortSettings.c_cflag &= ~CSIZE;
   SerialPortSettings.c_cflag |=  CS8;

   SerialPortSettings.c_cflag &= ~CRTSCTS;
   SerialPortSettings.c_cflag |= CREAD | CLOCAL;

   SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
   SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
   SerialPortSettings.c_oflag &= ~OPOST;

   SerialPortSettings.c_cc[VMIN] = 10;
   SerialPortSettings.c_cc[VTIME] = 0;

   tcflush(fd, TCIFLUSH);

   char read_buffer;
   int  bytes_read = 0;

   bytes_read = read(fd,&read_buffer,1);
       
   while(bytes_read != '\0')
   {
      printf("%c",read_buffer);
      bytes_read = read(fd,&read_buffer,1);
   }

   close(fd);
   return 0;
}
