#pragma once

#include "BaseMessage.hpp"

class TransmitData : public BaseMessage {
    public:
        uint16_t cmd_byte = 0x001E;
    private:
};