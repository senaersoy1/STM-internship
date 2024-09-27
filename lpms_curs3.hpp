#pragma once

#include "msg/TransmitDataMessage.hpp"

class LpmsCurs {
    public:
        LpmsCurs() = default;
        ~LpmsCurs() = default;
        void run();
    private:
        // Akış fonksiyonu
        bool configure();
        void listen();
        bool start_serial_port();

        TransmitData transmit_data{};
        bool should_run = false;
        // serial ports vs.
        int serial_port = -1;
        char* port = "/dev/ttyUSB0";
        int baudrate = 912600;

};