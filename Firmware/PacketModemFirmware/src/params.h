#ifndef _PARAMS_H
#define _PARAMS_H

#include <inttypes.h>
#include <avr/io.h>

#define EEPROM_HEADER 0xAA45
#define EEPROM_ADDRESS 0

/** Datatype for flash address */
typedef uint16_t flash_adr_t;

/** Datatype for EEPROM address */
typedef uint16_t eeprom_adr_t;

/** Datatype for return status of NVMCTRL operations */
typedef enum {
	NVM_OK    = 0, ///< NVMCTRL free, no write ongoing.
	NVM_ERROR = 1, ///< NVMCTRL operation retsulted in error
	NVM_BUSY  = 2, ///< NVMCTRL busy, write ongoing.
} nvmctrl_status_t;

typedef struct 
{
    uint16_t Header;
    uint8_t Checksum;
    uint8_t TxDelay;
    uint8_t Persistence;
    uint8_t SlotTime;
    uint8_t TxTail;
    uint8_t FullDuplex;
    uint8_t SetHardware;
    uint8_t Monitor;
    uint8_t MyCall[6];
    uint8_t MyId;
    uint8_t DestCall[6];
    uint8_t DestId;
    uint8_t Digi1[6];
    uint8_t Digi1Id;
    uint8_t Digi2[6];
    uint8_t Digi2Id;
    uint8_t StartMode;
    uint8_t IdText[25];
    uint8_t IdInterval;
    
} Packet_Params_t;


uint8_t FLASH_0_read_eeprom_byte(eeprom_adr_t eeprom_adr);
nvmctrl_status_t FLASH_0_write_eeprom_byte(eeprom_adr_t eeprom_adr, uint8_t data);
void FLASH_0_read_eeprom_block(eeprom_adr_t eeprom_adr, uint8_t *data, uint16_t size);
nvmctrl_status_t FLASH_0_write_eeprom_block(eeprom_adr_t eeprom_adr, uint8_t *data, uint16_t size);
int8_t FLASH_0_is_eeprom_ready();
uint8_t CalcEepromChecksum(Packet_Params_t *params);
int8_t CheckEepromChecksum(Packet_Params_t *params);
void SaveEepromContents(Packet_Params_t *params);
void ReadEepromContents(Packet_Params_t *params);

#endif /* _PARAMS_H */
