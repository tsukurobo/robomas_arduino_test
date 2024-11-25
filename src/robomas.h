#ifndef ROBOMAS_H
#define ROBOMAS_H
#include <SPI.h>
#include <mcp_can.h>

#define SS_Robomas_C610 10 // SPI通信に用いるピン

MCP_CAN CAN0(SS_Robomas_C610);

const int CAN_bps = CAN_1000KBPS; // 変更可
const int MCP_OSC = MCP_8MHZ;     // 回路上の発振子と合わせる

class C610
{
private:
public:
    // 動作電流値の最大値のデータ
    // これを超える電流値を流そうとしても反映されない。
    static uint16_t current_max[8] = {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}

    // C610に送信するデータ
    static uint8_t buf_0x200[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static uint8_t buf_0x1FF[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // 受け取ったモーターの角度を格納する変数
    // 格納されているのは0 ~ 8191の範囲の整数でそれぞれ0 ~ 2π radに対応している。
    static int received_angle[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // 受け取ったモーターの速度を格納する変数
    // 単位は rpm
    static int16_t received_velocity[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // 受け取ったモーターの実際のトルク電流を格納する変数
    // 単位は mA
    static int16_t received_torque_current[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // 初期化関数
    static void begin();

    // 電流値の最大値を設定する。
    static void setMaxCurrent(u_int C610_id, uint16_t max);

    // idに対して動作電流値を設定する。
    // 電流値の範囲は-10000mA~10000mA
    // PID制御しやすいようにdouble型を引数としているが16bitの整数に変換されるため小数点以下は反映されない。
    static void put(u_int C610_id, double current);

    // CAN通信でデータを送信。
    static void send();

    // 指示している電流値を読み取る。
    // 引数をtrueにすると16進数4桁で表示される。
    // falseにすると10進数で表示される。
    static void print(bool hex_flag = true);

    // C610からデータを受け取る
    static void receive();

    // モーターの角度を取得
    // 機械角度範囲:0~2π
    // 単位 rad
    static double getAngle(u_int C610_id);

    // モーター回転速度を取得
    // 単位 rpm
    static int16_t getVelocity(u_int C610_id);

    // 実際のトルク電流を取得
    // 単位 mA
    static int16_t getTorqueCurrent(u_int C610_id);
};

#endif