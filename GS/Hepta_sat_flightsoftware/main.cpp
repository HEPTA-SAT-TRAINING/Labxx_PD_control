#include "mbed.h"
#include "HEPTA_EPS.h"
#include "HEPTA_SENSOR.h"
#include "HEPTA_COM.h"

RawSerial pc(USBTX, USBRX, 9600);
HEPTA_EPS eps(p16, p26);
HEPTA_SENSOR sensor(p17,
                    p28, p27, 0x19, 0x69, 0x13,
                    p13, p14, p25, p24);
HEPTA_COM com(p9, p10, 9600);

bool mission_mode = false;
unsigned short telemetry_seq = 0;

void put_u16(unsigned char *packet, int *idx, unsigned short value)
{
    packet[(*idx)++] = (value >> 8) & 0xFF;
    packet[(*idx)++] = value & 0xFF;
}

void put_i16(unsigned char *packet, int *idx, short value)
{
    put_u16(packet, idx, (unsigned short)value);
}

short clamp_i16(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (short)value;
}

unsigned short clamp_u16(float value)
{
    if (value > 65535.0f) return 65535;
    if (value < 0.0f) return 0;
    return (unsigned short)value;
}

unsigned char calc_checksum(unsigned char *packet, int length)
{
    unsigned char sum = 0;

    for (int i = 1; i < length; i++) {
        sum += packet[i];
    }

    return sum;
}

void send_hk_packet(unsigned char packet_type,
                    unsigned char mode_byte,
                    float bt, float temp, float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float mx, float my, float mz)
{
    unsigned char packet[30];
    int idx = 0;

    telemetry_seq++;

    unsigned short bt_raw = clamp_u16(bt / (3.3f * 1.431f) * 4096.0f);

    short temp_raw = clamp_i16(temp * 10.0f);

    short ax_raw = clamp_i16(ax / 9.8f * 512.0f);
    short ay_raw = clamp_i16(ay / 9.8f * 512.0f);
    short az_raw = clamp_i16(az / 9.8f * 512.0f);

    short gx_raw = clamp_i16(gx * 2048.0f / 125.0f);
    short gy_raw = clamp_i16(gy * 2048.0f / 125.0f);
    short gz_raw = clamp_i16(gz * 2048.0f / 125.0f);

    short mx_raw = clamp_i16(mx);
    short my_raw = clamp_i16(my);
    short mz_raw = clamp_i16(mz);

    packet[idx++] = 0x7E;
    packet[idx++] = 0x01;
    packet[idx++] = packet_type;
    put_u16(packet, &idx, telemetry_seq);
    packet[idx++] = 23;

    packet[idx++] = mode_byte;

    put_u16(packet, &idx, bt_raw);
    put_i16(packet, &idx, temp_raw);

    put_i16(packet, &idx, ax_raw);
    put_i16(packet, &idx, ay_raw);
    put_i16(packet, &idx, az_raw);

    put_i16(packet, &idx, gx_raw);
    put_i16(packet, &idx, gy_raw);
    put_i16(packet, &idx, gz_raw);

    put_i16(packet, &idx, mx_raw);
    put_i16(packet, &idx, my_raw);
    put_i16(packet, &idx, mz_raw);

    packet[idx++] = calc_checksum(packet, idx);

    for (int i = 0; i < idx; i++) {
        com.putc(packet[i]);
    }
}

bool judge_power_save(float bt)
{
    return bt < 3.6f;
}

int main()
{
    int rcmd = 0, cmdflag = 0;
    float bt = 0, temp = 0;
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float mx = 0, my = 0, mz = 0;
    bool power_save_flag = false;

    while (1) {
        eps.vol(&bt);

        power_save_flag = judge_power_save(bt);

        if (power_save_flag) {
            ax = ay = az = 0;
            gx = gy = gz = 0;
            mx = my = mz = 0;
            temp = 0;

            send_hk_packet(0x12, 0x00,
                           bt, temp, ax, ay, az,
                           gx, gy, gz,
                           mx, my, mz);

            wait(1.0);
            continue;
        }

        com.xbee_receive(&rcmd, &cmdflag);

        if (cmdflag == true) {
            if (rcmd == 'a') {
                mission_mode = true;
            }

            if (rcmd == 'b') {
                mission_mode = false;
            }
        }

        cmdflag = 0;
        rcmd = 0;

        if (mission_mode == true) {
            sensor.temp_sense(&temp);
            sensor.sen_acc(&ax, &ay, &az);
            sensor.sen_gyro(&gx, &gy, &gz);
            sensor.sen_mag(&mx, &my, &mz);
        } else {
            sensor.temp_sense(&temp);
            ax = ay = az = 0;
            gx = gy = gz = 0;
            mx = my = mz = 0;
        }

        unsigned char packet_type = mission_mode ? 0x11 : 0x10;
        unsigned char mode_byte = mission_mode ? 0x01 : 0x00;

        send_hk_packet(packet_type, mode_byte,
                       bt, temp, ax, ay, az,
                       gx, gy, gz,
                       mx, my, mz);

        wait(1.0);
    }
}