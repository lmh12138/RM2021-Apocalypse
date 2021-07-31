#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "stdint.h"
#include "can.h"

#define SUPER_CAP_ID 0x211

struct SuperCap_t
{
    float voltage_input_fdb;
    float vlotage_cap_fdb;
    float current_input_fdb;
    float power_set_fdb;
    float power_set;
};

extern struct SuperCap_t spuer_cap;

void SuperCap_init(void);
void SuperCap_Encoder(struct SuperCap_t *cap, uint8_t *RecvData);
void Can_tx_supercap(CAN_HandleTypeDef *hcanx, float power_set);
void SuperCap_thread(void);

#endif
