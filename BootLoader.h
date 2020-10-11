#ifndef __BOOTLOADER_H__
#define __BOOTLOADER_H__

#include <GenericTypeDefs.h>

#define PROGRAM_FLASH_END_ADRESS (0x9D000000+BMXPFMSZ-1)
/* APP_FLASH_BASE_ADDRESS and APP_FLASH_END_ADDRESS reserves program Flash for the application*/
/* Rule:
 		1)The memory regions kseg0_program_mem, kseg0_boot_mem, exception_mem and
 		kseg1_boot_mem of the application linker script must fall with in APP_FLASH_BASE_ADDRESS
 		and APP_FLASH_END_ADDRESS

 		2)The base address and end address must align on  4K address boundary */

#define APP_FLASH_BASE_ADDRESS 	0x9D006000
#define APP_FLASH_END_ADDRESS   PROGRAM_FLASH_END_ADRESS

/* Address of  the Flash from where the application starts executing */
/* Rule: Set APP_FLASH_BASE_ADDRESS to _RESET_ADDR value of application linker script*/

// For PIC32MX1xx and PIC32MX2xx Controllers only
#define USER_APP_RESET_ADDRESS 	(0x9D006000 + 0x1000)

#define REC_FLASHED 0
#define REC_NOT_FOUND 1
#define REC_FOUND_BUT_NOT_FLASHED 2

typedef struct
{
    UINT8 *start;
    UINT8 len;
    UINT8 status;
}T_REC;

typedef struct 
{
	UINT8 RecDataLen;
	DWORD_VAL Address;
	UINT8 RecType;
	UINT8* Data;
	UINT8 CheckSum;	
	DWORD_VAL ExtSegAddress;
	DWORD_VAL ExtLinAddress;
}T_HEX_RECORD;	


#define DATA_RECORD 		0
#define END_OF_FILE_RECORD 	1
#define EXT_SEG_ADRS_RECORD 2
#define EXT_LIN_ADRS_RECORD 4

#endif
