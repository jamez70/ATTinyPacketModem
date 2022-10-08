#include  <inttypes.h>
//#include <EEPROM.h>
#include <avr/eeprom.h>
#include <string.h>

#include "params.h"


void serprintln(const char *format, ...);

uint8_t CalcEepromChecksum(Packet_Params_t *params)
{
    uint8_t chk;
    uint16_t pos;
    uint8_t *data;
    // Clear checksum first
    params->Checksum = 0;
    data = (uint8_t *)params;

    chk=0;
    for (pos=0;pos<sizeof(Packet_Params_t);pos++)
    {
        chk += data[pos];
    }
    chk = ~chk;
    params->Checksum = chk;
    return chk;
}

int8_t CheckEepromChecksum(Packet_Params_t *params)
{
    uint8_t tempchk;
    int8_t val;

    val = 0;
    tempchk = params->Checksum;
    
    if (CalcEepromChecksum(params) != tempchk)
    {
        val = -1;
    }
    return val;
}

void SaveEepromContents(Packet_Params_t *params)
{
    CalcEepromChecksum(params);
    eeprom_write_block((uint8_t*)params, EEPROM_ADDRESS,sizeof(Packet_Params_t) );
//    FLASH_0_write_eeprom_block(EEPROM_ADDRESS, (uint8_t)params, sizeof(Packet_Params_t));
}

void SetEepromDefaults(Packet_Params_t *params)
{
    serprintln("Setting EEPROM defaults");
    params->Header = EEPROM_HEADER;
    params->FullDuplex = 0;
    params->Checksum = 0;
    params->Persistence = 10;
    params->SetHardware = 0;
    params->SlotTime = 1;
    params->TxDelay = 30;
    params->TxTail = 10;
    strncpy((char*)params->MyCall, "NOCALL", 6);
    strncpy((char*)params->DestCall, "NOCALL", 6);
    memset(params->Digi1, 32, 6);
    params->Digi1Id = 0;
    memset(params->Digi2, 32, 6);
    params->Digi2Id = 0;
    params->MyId = 0;
    params->DestId = 0;
    params->Monitor = 0;
    params->IdText[0] = 0;
    params->IdInterval = 0;
    params->StartMode = 0;
}

void ReadEepromContents(Packet_Params_t *params)
{
    Packet_Params_t tmpParams;
    int8_t result;

    eeprom_read_block( (uint8_t*)&tmpParams, EEPROM_ADDRESS,sizeof(Packet_Params_t) );
    result = CheckEepromChecksum(&tmpParams);
    if (result != 0)   
    {
        serprintln("EEPROM checksum error");
        SetEepromDefaults(&tmpParams);
        SaveEepromContents(&tmpParams);
    }
    memcpy((uint8_t *)params, (uint8_t *)(&tmpParams), sizeof(Packet_Params_t));
}