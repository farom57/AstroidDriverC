#include "messages.h"

float char2float(char const *buf){
    uint32_t tmp = ((uint8_t)buf[0]<<24) + ((uint8_t)buf[1] << 16) + ((uint8_t)buf[2] << 8) + (uint8_t)buf[3];
    return *(float*)(&tmp);
}

int32_t char2long(char const *buf){
    return ((uint8_t)buf[0]<<24) + ((uint8_t)buf[1] << 16) + ((uint8_t)buf[2] << 8) + (uint8_t)buf[3];
}

uint16_t char2uint(char const *buf){
    return (buf[0]<<8) + buf[1];
}


Status_message::Status_message(){
    ms_count = 0;
    step_ha = 0;
    step_de = 0;
    ustep_ha = 0;
    ustep_de = 0;
    move_speed_ha = 0;
    move_speed_de = 0;
    power_ha = 0;
    power_de = 0;
    power_aux_1 = 0;
    power_aux_2 = 0;
    power_aux_3 = 0;
    // byte 42 not used
    step_focus = 0;
    ustep_focus = 0;
    move_speed_focus = 0;
    checksum = 0;
}

bool Status_message::set_buf(char const *buf){
    if(!verify(buf)){
        return false;
    }


    ms_count = char2long(buf);
    step_ha = char2long(buf+4);
    step_de = char2long(buf+8);
    ustep_ha = char2float(buf+12);
    ustep_de = char2float(buf+16);
    move_speed_ha = char2float(buf+20);
    move_speed_de = char2float(buf+24);
    power_ha = char2float(buf+28);
    power_de = char2float(buf+32);
    power_aux_1 = char2uint(buf+36);
    power_aux_2 = char2uint(buf+38);
    power_aux_3 = char2uint(buf+40);
    // byte 42 not used
    step_focus = char2long(buf+43);
    ustep_focus = char2float(buf+47);
    move_speed_focus = char2float(buf+51);
    checksum = buf[55];

    return true;
}



bool Status_message::verify(char const *buf){
    uint8_t checksum_calculated = 0;
    for(int i = 0; i < 55; i++){
        checksum_calculated += buf[i];
    }
    return checksum_calculated == (0xFF & buf[55]);
}


double Status_message::getDE(){
    return (double) step_de + (double)ustep_de/1024.;
}

double Status_message::getHA(){
    return (double) step_ha + (double)ustep_ha/1024.;
}


Cmd_message::Cmd_message(){
    speed_ha = 0;
    speed_de = 0;
    power_ha = 1;
    power_de = 1;
    power_aux_1 = 0;
    power_aux_2 = 0;
    power_aux_3 = 0;
    speed_focus = 0;
    power_focus = 0;
}

void float2char(float val, char *buf){
    uint32_t val2 = *(uint32_t*)(&val);
    buf[0] = (val2 >> 24) & 0xFF;
    buf[1] = (val2 >> 16) & 0xFF;
    buf[2] = (val2 >> 8) & 0xFF;
    buf[3] = val2 & 0xFF;
}

void uint2char(uint16_t val, char *buf){
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
}

void Cmd_message::get_bytes(char *buf){
    float2char(speed_ha, buf);
    float2char(speed_de, buf+4);
    float2char(power_ha, buf+8);
    float2char(power_de, buf+12);
    uint2char(power_aux_1, buf+16);
    uint2char(power_aux_2, buf+18);
    uint2char(power_aux_3, buf+20);
    buf[22] = 0;
    float2char(speed_focus, buf+23);
    float2char(power_focus, buf+27);

    uint8_t checksum = 0;
    for(int i = 0; i < 31; i++){
        checksum += buf[i];
    }
    buf[31] = checksum;
}
