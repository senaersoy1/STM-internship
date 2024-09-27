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

enum class State {
    HEADER,
    SENSOR_ID,
    COMMAND,
    LENGTH,
    DATA,
    LRC,
    TERMINATION
};

enum class ParserState {
    NEED_DATA,
    COMPLETED,
    ERROR
};

class LPBusParser{   
   public:   
    uint16_t m_sensor_id = 0;
    uint16_t m_command = 0; 
    uint16_t m_length = 0;  
    uint16_t m_termination = 0;
    uint8_t m_data[72]; 
    uint16_t m_lrc= 0;  
    uint8_t buffer_position = 0;
    uint8_t byte_counter = 0;

    State m_state = State::HEADER;  
    ParserState parser_state = ParserState::NEED_DATA; 

    struct Vector3f {
            float x, y, z;
        };

    struct Vector4f {
            float x, y, z, t;
        };

    struct SensorData{
        uint32_t timestamp;
        Vector3f AccelerometerCalibrated;
        Vector3f GyroAlignmentBiasCalibrated;
        Vector3f MagnetometerCalibrated;
        Vector4f Quaternion;
        Vector3f Euler;
        float Temperature;
    };

    SensorData sensorData;

    void reset(){
        m_sensor_id = 0;
        m_command = 0;
        m_length = 0;
        m_lrc = 0;
        m_termination = 0;
        std::memset(m_data, 0, sizeof(m_data));
        buffer_position = 0;
        byte_counter = 0;
        m_state = State::HEADER;
        parser_state = ParserState::NEED_DATA;
    }

    uint16_t calculateLRC() {

        uint16_t lrc = m_sensor_id + m_command + m_length;

        for (int i = 0; i < m_length; ++i){
            lrc += static_cast<uint16_t>(m_data[i]);
        }

        return lrc;  
    }

    void parseByte(uint8_t byte){
        switch (m_state) {
            case State::HEADER:
                parser_state = ParserState::NEED_DATA;
                if (byte == 0x3A) {  
                    m_state = State::SENSOR_ID;
                }
                break;

            case State::SENSOR_ID:
                if (byte_counter == 0){
                    m_sensor_id = static_cast<uint16_t>(byte);
                    byte_counter++;
                }else{
                    m_sensor_id |= (static_cast<uint16_t>(byte) << 8);
                    byte_counter = 0;
                    if (m_sensor_id != 0x0001){
                        m_state = State::HEADER;
                        reset();
                    } else
                        m_state = State::COMMAND;
                }
                break;

            case State::COMMAND:
                if (byte_counter == 0){
                    m_command = static_cast<uint16_t>(byte);
                    byte_counter++;
                }else{
                    m_command |= (static_cast<uint16_t>(byte) << 8);
                    byte_counter = 0;
                    if (m_command != 0x0009){
                        m_state = State::HEADER;
                        reset();
                    } else
                        m_state = State::LENGTH;
                }
                break;

            case State::LENGTH:
                if (byte_counter == 0){
                    m_length = static_cast<uint16_t>(byte);
                    byte_counter++;
                }else{
                    m_length |= (static_cast<uint16_t>(byte) << 8);
                    byte_counter = 0;
                    if (m_length > 0)
                        m_state = State::DATA;
                    else if ((m_length > 0) && (m_length > 72)){
                        m_state = State::HEADER;
                        parser_state = ParserState::ERROR;
                        reset();
                    }
                    else
                        m_state = State::LRC;
                    }
                break;

            case State::DATA:
                if (buffer_position < sizeof(m_data)){
                    m_data[buffer_position] = byte;
                    buffer_position++;

                    if (buffer_position == m_length){

                        buffer_position = 0;

                        int data_index = 0;

                        std::memcpy(&sensorData.timestamp, &m_data[data_index], sizeof(uint32_t));
                        data_index += sizeof(uint32_t);

                        std::memcpy(&sensorData.AccelerometerCalibrated, &m_data[data_index], sizeof(Vector3f));
                        data_index += sizeof(Vector3f);

                        std::memcpy(&sensorData.GyroAlignmentBiasCalibrated, &m_data[data_index], sizeof(Vector3f));
                        data_index += sizeof(Vector3f);

                        std::memcpy(&sensorData.MagnetometerCalibrated, &m_data[data_index], sizeof(Vector3f));
                        data_index += sizeof(Vector3f);

                        std::memcpy(&sensorData.Quaternion, &m_data[data_index], sizeof(Vector4f));
                        data_index += sizeof(Vector4f);

                        std::memcpy(&sensorData.Euler, &m_data[data_index], sizeof(Vector3f));
                        data_index += sizeof(Vector3f);

                        std::memcpy(&sensorData.Temperature, &m_data[data_index], sizeof(float));
                        m_state = State::LRC;
                    }
                }else
                    reset();  
                break;
            case State::LRC:
                if (byte_counter == 0) {

                    m_lrc = static_cast<uint16_t>(byte);
                    byte_counter++;
                } else {
                    
                    m_lrc |= (static_cast<uint16_t>(byte) << 8);
                    byte_counter = 0;
                    uint16_t calculatedLRC = calculateLRC();

                    if (calculatedLRC > 0 && calculatedLRC == m_lrc) {

                        m_state = State::TERMINATION;
                    } else {
                        reset();
                        parser_state = ParserState::ERROR;
                        m_state = State::HEADER;
                        std::cout << "Calculated LRC: " << calculatedLRC << "  Received LRC: " << m_lrc << std::endl;
                    }

                }
                break;
            case State::TERMINATION:
                if (byte_counter == 0) {

                    m_termination = static_cast<uint16_t>(byte);
                    byte_counter++;
                } else {

                    m_termination |= (static_cast<uint16_t>(byte) << 8);
                    byte_counter = 0;

                    if (m_termination == 0x0A0D) {
                        reset();
                        parser_state = ParserState::COMPLETED;
                        std::cout << "TERMINATION" << std::endl;
                    }
                    else {
                        m_state = State::HEADER;
                        reset();
                    }
                }
                break;
        }
    }
};

int main() {
    
    const char* portname = "/dev/ttyUSB0";
    int fd = open(portname, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "FAILED";
        return -1;
    }
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
    cfsetispeed(&tty, 912600);
    cfsetospeed(&tty, 912600);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting termios attributes" << std::endl;
        return -1;
    }

    // GOTO_COMMAND_MODE command
    std::vector<unsigned char> GOTO_COMMAND_MODE = {0x3A, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x07, 0x00, 0x0D, 0x0A};
    writeCommand(fd, GOTO_COMMAND_MODE);
    //tcdrain(fd);
    readResponse(fd, 11);

    // SET_TRANSMIT_DATA command
    std::vector<unsigned char> SET_TRANSMIT_DATA = {0x3A, 0x01, 0x00, 0x1E, 0x00, 0x04, 0x00, 0x82, 0x1A, 0x01, 0x00, 0xC0, 0x00, 0x0D, 0x0A};
    writeCommand(fd, SET_TRANSMIT_DATA);
    //tcdrain(fd);
    readResponse(fd, 11);

    // GOTO_STREAMING_MODE
    std::vector<unsigned char> GOTO_STREAMING_MODE = {0x3A, 0x01, 0x00, 0x07, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0D, 0x0A};
    writeCommand(fd, GOTO_STREAMING_MODE);
    //tcdrain(fd);
    readResponse(fd, 11);    
    
    LPBusParser parser;

    while (true) {

        int available_bytes = get_available_bytes(fd);

        if (available_bytes > 0) {

            std::vector<uint8_t> temp_buffer(available_bytes);
            int bytes_read = read(fd, temp_buffer.data(), available_bytes);

            for (auto i = 0; i < bytes_read; i++) {
                
                uint8_t byte = temp_buffer[i];
                parser.parseByte(byte);

                if (parser.parser_state == ParserState::COMPLETED) {

                    std::cout << "Parsing completed successfully." << std::endl;

                    std::cout << "Accelerometer Calibrated Data: ("
                            << parser.sensorData.AccelerometerCalibrated.x << ", "
                            << parser.sensorData.AccelerometerCalibrated.y << ", "
                            << parser.sensorData.AccelerometerCalibrated.z << ")" << std::endl;

                    std::cout << "Gyro Aliignment Bias Calibrated Data: ("
                            << parser.sensorData.GyroAlignmentBiasCalibrated.x << ", "
                            << parser.sensorData.GyroAlignmentBiasCalibrated.y << ", "
                            << parser.sensorData.GyroAlignmentBiasCalibrated.z << ")" << std::endl;

                    std::cout << "Magnetometer Calibrated Data: ("
                            << parser.sensorData.MagnetometerCalibrated.x << ", "
                            << parser.sensorData.MagnetometerCalibrated.y << ", "
                            << parser.sensorData.MagnetometerCalibrated.z << ")" << std::endl;

                    std::cout << "Euler Data: ("
                            << parser.sensorData.Euler.x << ", "
                            << parser.sensorData.Euler.y << ", "
                            << parser.sensorData.Euler.z << ")" << std::endl;

                    std::cout << "Quaternion Data: ("
                            << parser.sensorData.Quaternion.x << ", "
                            << parser.sensorData.Quaternion.y << ", "
                            << parser.sensorData.Quaternion.z << ", "
                            << parser.sensorData.Quaternion.t << ")" << std::endl;

                    std::cout << "Temperature: " << parser.sensorData.Temperature << std::endl;

                    std::cout << "Timestamp: " << parser.sensorData.timestamp << std::endl;

                    std::cout << "Timestamp: " << parser.sensorData.timestamp * 0.002f << " seconds" << std::endl;

                }else if (parser.parser_state == ParserState::ERROR) 
                    std::cerr << "Parsing error occurred." << std::endl;  
                              
            }
        }
    }

    close(fd);
    return 0;
}

