#include <numbers>
#include "robomas.h"

void C610::begin()
{
    CAN0.begin(MCP_ANY, CAN_bps, MCP_OSC);
    if (CAN0.begin(MCP_ANY, CAN_bps, MCP_OSC) == CAN_OK)
    {
        Serial.println("Success : MCP2515 Initialization");
    }
    else
    {
        Serial.println("Fail : MCP2515 Initialization");
    }
    delay(100);
    CAN0.setMode(MCP_NORMAL);
}

void setMaxCurrent(u_int C610_id, uint16_t max)
{
    if (1 <= C610_id && C610_id <= 8 && 0 <= max && max <= 10000)
    {
        this->current_max[C610_id - 1] = max;
        return;
    }
    Serial.println { "Fail : id or max is out of range." }
}

void C610::put(u_int C610_id, double current)
{
    // 想定外の電流値の場合は何もしない。
    if (abs(current) > this->current_max[C610_id + 1])
    {
        Serial.println("Fail : This current value is out of range.") return;
    }
    // currentを整数値に変換して上位8bitと下位8bitに分ける。
    int16_t converted = static_cast<int16_t>(current);
    uint8_t high_byte = (converted >> 8) & 0xFF; // 上位8ビット
    uint8_t low_byte = converted & 0xFF;         // 下位8ビット

    if (1 <= C610_id && C610_id <= 4)
    {
        this->buf_0x200[(C610_id - 1) * 2] = high_byte;
        this->buf_0x200[(C610_id - 1) * 2 + 1] = low_byte;
        return;
    }
    if (5 <= C610_id && C610_id <= 8)
    {
        this->buf_0x1FF[(C610_id - 5) * 2] = high_byte;
        this->buf_0x1FF[(C610_id - 5) * 2 + 1] = low_byte;
        return;
    }

    Serial.println("Fail : This C610_id is out of range.")
}

void C610::send()
{
    // データの送信
    CAN0.sendMsgBuf(0x200, 0, 8, this->buf_0x200);
    CAN0.sendMsgBuf(0x1FF, 0, 8, this->buf_0x1FF);
}

void C610::print(bool hex_flag = true)
{
    for (int i = 1; i < 9; i++)
    {
        int16_t value;
        uint8_t high_byte;
        uint8_t low_byte;
        if (1 <= i && i <= 4)
        {
            high_byte = this->buf_0x200[(i - 1) * 2];
            low_byte = this->buf_0x200[(i - 1) * 2 + 1];
        }
        if (5 <= i && i <= 8)
        {
            high_byte = this->buf_0x200[(i - 5) * 2];
            low_byte = this->buf_0x200[(i - 5) * 2 + 1];
        }
        value = (high_byte << 8) + low_byte;
        if (hex_flag)
            Serial.print(value, HEX);
        else
            Serial.print(value)
                Serial.print(" ");
    }
    Serial.println();
}

void C610::receive()
{
    if (CAN_MSGAVAIL == CAN0.checkReceive())
    {
        for (int i = 1; i < 9; i++)
        {
            uint8_t len = 0;
            uint8_t data[8];
            unsigned long int rx_message;
            CAN0.readMsgBuf(&rx_message, &len, data);
            if (rx_message == 0x200 + i)
            {
                this->received_angle[i - 1] == (data[0] << 8) + data[1];
                this->received_velocity[i - 1] == (data[2] << 8) + data[3];
                this->received_torque_current[i - 1] == (data[4] << 8) + data[5];
            }
        }
    }
}

double C610::getAngle(u_int C610_id)
{
    if (C610_id < 1 || 8 < C610_id)
    {
        Serial.print("Fail : id is out of range.");
        return -1;
    }
    double angle = this->received_angle[C610_id - 1] / 8191 * 2 * std::numbers::pi;
    return angle;
}

int16_t C610::getVelocity(u_int C610_id)
{
    if (C610_id < 1 || 8 < C610_id)
    {
        Serial.print("Fail : id is out of range.");
        return -1;
    }
    int16_t velocity = this->received_velocity[C610_id - 1];
    return velocity;
}

int16_t C610::getAngle(u_int C610_id)
{
    if (C610_id < 1 || 8 < C610_id)
    {
        Serial.print("Fail : id is out of range.");
        return -1;
    }
    int16_t torque_current = this->received_torque_current[C610_id - 1];
    return torque_current;
}