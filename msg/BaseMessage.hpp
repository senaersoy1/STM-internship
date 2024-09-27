#pragma once

class BaseMessage {
    public:
        void serialize();
        void deserialize();
        int calculateLRC();
        uint16_t m_header = 0x3A;
        // ...
};