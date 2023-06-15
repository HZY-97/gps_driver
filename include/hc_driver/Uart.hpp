#include <string>
using namespace std;
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>
#include <memory.h>

#define ReadBuffLength 8192

class Comnav_Uart
{
public:
    int fd;
    Comnav_Uart() : Baudrate(115200), opened(false){};
    Comnav_Uart(string Port) : Port(Port), Baudrate(115200), opened(false) { open(); };
    Comnav_Uart(string Port, int Baudrate) : Port(Port), Baudrate(Baudrate), opened(false) { open(); };

    void open()
    {
        fd = ::open(Port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        set_interface_attribs(fd, BaudrateIntToTermiosBaud(Baudrate), 0);
        set_blocking(fd, 0);
        opened = (fd != -1);
    };
    void close()
    {
        ::close(fd);
        fd = -1;
        opened = false;
    };

    string read()
    {
        if(not opened) return "";
        char buff[ReadBuffLength];
        int nread = ::read(fd, (unsigned char *)buff, ReadBuffLength);
        return string().append(buff, nread);
    }
    int write(string buff) { return ::write(fd, (unsigned char *)buff.c_str(), buff.length()); };

    string Port;
    int Baudrate;

    void Set(string Port, int Baudrate)
    {
        if (this->Port == Port && this->Baudrate == Baudrate)
            return;
        this->Port = Port;
        this->Baudrate = Baudrate;
        close(); open();
    }
    void Set(int Baudrate)
    {
        if (this->Baudrate == Baudrate)
            return;
        this->Baudrate = Baudrate;
        close(); open();
    }

    int BaudrateIntToTermiosBaud(int Baudrate)
    {
        if (Baudrate == 57600)
            return B57600;
        if (Baudrate == 115200)
            return B115200;
        if (Baudrate == 230400)
            return B230400;
        if (Baudrate == 460800)
            return B460800;
        if (Baudrate == 500000)
            return B500000;
        if (Baudrate == 576000)
            return B576000;
        if (Baudrate == 921600)
            return B921600;
        return 115200;
    }

    int set_interface_attribs(int fd, int speed, int parity)
    {
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0)
        {
            //printf("Uart error %d from tcgetattr\r\n", errno);
            return -1;
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK; // disable break processing
        tty.c_lflag = 0;        // no signaling chars, no echo,
        // no canonical processing
        tty.c_oflag = 0;     // no remapping, no delays
        tty.c_cc[VMIN] = 0;  // read doesn't block
        tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            printf("error %d from tcsetattr", errno);
            return -1;
        }
        return 0;
    }

    void set_blocking(int fd, int should_block)
    {
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(fd, &tty) != 0)
        {
            //printf("error %d from tggetattr", errno);
            return;
        }

        tty.c_cc[VMIN] = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

        if (tcsetattr(fd, TCSANOW, &tty) != 0);
            //printf("error %d setting term attributes", errno);
    }

    bool opened;
};