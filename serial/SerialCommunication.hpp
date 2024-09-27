#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <vector>
#include <cstring>
#include <cstdint>
#include <sys/ioctl.h> 
#include <math.h>
#include <numeric> 

void configureSerialPort(int fd) {
    struct termios tty;

    tcgetattr(fd, &tty);

    tty.c_oflag &= ~ONLCR;

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;             // 8 data bits
    tty.c_cflag &= ~PARENB;         // No parity bit
    tty.c_cflag &= ~CSTOPB;         // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_iflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_iflag &= ~OPOST;

    // Set timeout and minimum characters to read
    tty.c_cc[VTIME] = 0;            // Timeout in deciseconds (0.1 seconds)
    tty.c_cc[VMIN] = 0;            // Minimum characters to read
    
    // Set input and output baud rate
    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting termios attributes" << std::endl;
        return;
    }
}

void writeCommand(int fd, const std::vector<unsigned char>& cmd) {
    tcflush(fd, TCIFLUSH);
    ssize_t x = write(fd, cmd.data(), cmd.size());
    if (-1 == x) {
        std::cerr << "Error writing" << std::endl;
    }
}

void readResponse(int fd, int read_size) {
    std::vector<unsigned char> buffer(read_size);
    int bytes_read = read(fd, buffer.data(), buffer.size());

    if (bytes_read == -1) {
        std::cerr << "Error reading" << std::endl;
        return;
    }

    // Print the received data in hex format
    std::cout << "Reply= ";
    for (int i = 0; i < bytes_read; ++i) {
        printf("%02X ", buffer[i]);
    }
    std::cout << std::endl;
}

int get_available_bytes(int fd){
    int bytes_available;
        // ioctl() with FIONREAD to get the number of bytes available for reading
    if (ioctl(fd, FIONREAD, &bytes_available) == -1) {
        std::cerr << "Error in ioctl()" << std::endl;
        return -1;
    }
    return bytes_available;
}
