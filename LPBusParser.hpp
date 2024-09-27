#pragma once
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
        float timestampInSeconds = (static_cast<float>(timestamp) * 0.002f);
        Vector3f AccelerometerCalibrated;
        Vector3f GyroAlignmentBiasCalibrated;
        Vector3f MagnetometerCalibrated;
        Vector4f Quaternion;
        Vector3f Euler;
        float Temperature;
    };

    SensorData sensorData;

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
                    if (m_sensor_id != 0x0001)
                        m_state = State::HEADER;
                    else
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
                    if (m_command != 0x0009)
                        m_state = State::HEADER;
                    else
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
                    else if ((m_length > 0) && (m_length > 72))
                        m_state = State::HEADER;
                    else
                        m_state = State::LRC;
                    }
                    std::cout << "Data Length: " << m_length << std::endl;
                break;

            case State::DATA:
                if (buffer_position < sizeof(m_data)){
                    m_data[buffer_position] = byte;
                    buffer_position++;

                    if (buffer_position == m_length){

                        std::cout << "Data Received!!" << std::endl;
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
                    parser_state = ParserState::NEED_DATA;   
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

                        m_state = State::HEADER;
                    }
                    std::cout << "Calculated LRC: " << calculatedLRC << "  Received LRC: " << m_lrc << std::endl;

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

                        parser_state = ParserState::COMPLETED;
                        std::cout << "TERMINATION" << std::endl;
                    }

                    m_state = State::HEADER;
                }
                break;
        }
    }
};
