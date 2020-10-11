/********************************************************************
 FileName:          main.c
 Dependencies:
 Processor:         PIC32MX795F512L
 Hardware:		
 Complier:          Microchip XC32
 Company:           WGP
********************************************************************/
#include <xc.h>

#include <string.h>
#include <stdio.h>

#include "sd/fsio.h"
#include "NVMem.h"
#include "BootLoader.h"
#include "HardwareProfile.h"

#define PROGRAM_FILE_NAME       "pgm.hex"
#define PROGRAM_FILE_NAME_BK    "pgmbk.hex"
#define VERIFY_PROGRAM

// *****************************************************************************
// *****************************************************************************
// Device Configuration Bits (Runs from Aux Flash)
// *****************************************************************************
// *****************************************************************************
// Configuring the Device Configuration Registers

// Configuring the Device Configuration Registers
// 80Mhz Core/Periph, Pri Osc w/PLL, Write protect Boot Flash
#pragma config FPLLODIV = DIV_1
#pragma config FPLLMUL = MUL_20
#pragma config DEBUG    = OFF       // Background Debugger disabled
#pragma config FPLLIDIV = DIV_2
#pragma config FWDTEN = OFF
#pragma config FPBDIV = DIV_1
#pragma config POSCMOD = XT
#pragma config FNOSC = PRIPLL
#pragma config CP = OFF
// Boot write protect: OFF
#pragma config BWP = OFF
// external PHY in RMII/alternate configuration
#pragma config FMIIEN = OFF
#pragma config FETHIO = OFF
// USB
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider

/******************************************************************************
Macros used in this file
*******************************************************************************/
#define AUX_FLASH_BASE_ADRS             (0x7FC000)
#define AUX_FLASH_END_ADRS              (0x7FFFFF)
#define DEV_CONFIG_REG_BASE_ADDRESS     (0xF80000)
#define DEV_CONFIG_REG_END_ADDRESS      (0xF80012)

/******************************************************************************
Global Variables
*******************************************************************************/
FSFILE * myFile;
//BYTE myData[512];
//size_t numBytes;
//UINT readBytes;

unsigned int    ledCount;

UINT8 asciiBuffer[80];
UINT8 hexRec[100];

#define READ_RETRY 100

/****************************************************************************
Function prototypes
*****************************************************************************/
void JumpToApp(void);
BOOL ValidAppPresent(void);
void EraseFlash(void);
int WriteHexRecord2Flash(UINT8* HexRecord);
int CheckHexRecord(UINT8* HexRecord);
void ConvertAsciiToHex(UINT8* asciiRec, UINT8* hexRec);
int readLine(void *ptr, size_t max_len, FSFILE *stream);
void BlinkLed(unsigned int period)
{
    if( !((++ledCount) % period) ) mLED ^= 1;
}

void Error( unsigned int err )
{
    volatile unsigned long delay_count;
    unsigned int pulse_count;
    unsigned int loop_count;
    /* Valores de err:
     * 1: No hay SD y no hay programa en el micro
     * 2: No hay programa en la SD ni en el micro
     * 3: El programa en la SD no es valido y no hay programa en el micro
     * 4: Error de lectura cargando programa
     * 5: Error de checksum cargando programa
     */
    mLED = 0;
    pulse_count = err;
    loop_count = 10;
    for(delay_count = 10000000; delay_count > 0; delay_count--);
    while(1)
    {
        for(delay_count = 10000000; delay_count > 0; delay_count--);
        if(pulse_count)
        {
            mLED = 1;
            pulse_count--;
        }
        loop_count--;
        for(delay_count = 10000000; delay_count > 0; delay_count--);
        mLED = 0;
        if( !loop_count)
        {
            pulse_count = err;
            loop_count = 10;
        }
    }
}

/*****************************************************************************/
int main(void)
{
    volatile UINT i;
    myFile = NULL;
    long bytecount;
    int byteread;
    int readretry;
    int validHex = 0;
        
    // Setup configuration
    //(void)SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    /* Inicializo como salida en '0' los pines que no se usan */
    /*
        RE5	3         RE6	4         RE7	5         RA9	28
        RA10	29        RB11	35        AECRS	41        AECOL	42
        AETXD3	43        AETXD2 44       *vusb	54        *vbus	55
        RD2	77        RD3	78        RD12	79        RD13	80
        RF0	87        RF1	88        RG1	89        RG0	90
        RE0	93        RE1	94        RG14	95        RG12	96
        RG13	97        RE2	98        RE3	99        RE4	100
     */
//    TRISEbits.TRISE5 = 0;       PORTEbits.RE5 = 0;
//    TRISEbits.TRISE6 = 0;       PORTEbits.RE6 = 0;
//    TRISEbits.TRISE7 = 0;       PORTEbits.RE7 = 0;
//    TRISAbits.TRISA9 = 0;       PORTAbits.RA9 = 0;
//    TRISAbits.TRISA10 = 0;      PORTAbits.RA10 = 0;
//    TRISBbits.TRISB11 = 0;      PORTBbits.RB11 = 0;
//    TRISBbits.TRISB12 = 0;      PORTBbits.RB12 = 0;
//    TRISBbits.TRISB13 = 0;      PORTBbits.RB13 = 0;
//    TRISBbits.TRISB14 = 0;      PORTBbits.RB14 = 0;
//    TRISBbits.TRISB15 = 0;      PORTBbits.RB15 = 0;
//    TRISDbits.TRISD2 = 0;       PORTDbits.RD2 = 0;
//    TRISDbits.TRISD3 = 0;       PORTDbits.RD3 = 0;
//    TRISDbits.TRISD12 = 0;      PORTDbits.RD12 = 0;
//    TRISDbits.TRISD13 = 0;      PORTDbits.RD13 = 0;
//    TRISFbits.TRISF0 = 0;       PORTFbits.RF0 = 0;
//    TRISFbits.TRISF1 = 0;       PORTFbits.RF1 = 0;
//    TRISGbits.TRISG1 = 0;       PORTGbits.RG1 = 0;
//    TRISGbits.TRISG0 = 0;       PORTGbits.RG0 = 0;
    TRISEbits.TRISE0 = 0;       PORTEbits.RE0 = 1;      /* Power GSM Modem */
//    TRISEbits.TRISE1 = 0;       PORTEbits.RE1 = 0;
//    TRISGbits.TRISG14 = 0;      PORTGbits.RG14 = 0;
//    TRISGbits.TRISG12 = 0;      PORTGbits.RG12 = 0;
//    TRISGbits.TRISG13 = 0;      PORTGbits.RG13 = 0;
//    TRISEbits.TRISE2 = 0;       PORTEbits.RE2 = 0;
//    TRISEbits.TRISE3 = 0;       PORTEbits.RE3 = 0;
//    TRISEbits.TRISE4 = 0;       PORTEbits.RE4 = 0;

    /* Configuro el led de status */
    TRISDbits.TRISD6 = 0; LATDbits.LATD6 = 0;
    
    ledCount = 0;

    // Initialize the File System
    if(!FSInit())
    {
        /* Si no puedo iniciaizar la SD trato de saltar al programa */
        if(ValidAppPresent())
        {
            JumpToApp();
        }
        else
        {
            //Indicate error and stay in while loop.
            Error(1);
        }
    }         

    /* Siempre que haya un archivo en la SD lo cargo */
    myFile = FSfopen(PROGRAM_FILE_NAME, "r");

    if(myFile == NULL)// Make sure the file is present.
    {
        if(ValidAppPresent())
        {
            JumpToApp();
        }
        else
        {
            /* Trato de abrir un bacup */
            myFile = FSfopen(PROGRAM_FILE_NAME_BK, "r");

            if(myFile == NULL)// Make sure the file is present.
            {
                //Indicate error and stay in while loop.
                Error(2);
            }
        }
    }     

#ifdef VERIFY_PROGRAM
    /* Verifico el archivo HEX */
    bytecount = 0;
    readretry = READ_RETRY;
    while(readretry && bytecount < myFile->size && validHex == 0)
    {
        while((byteread = readLine(asciiBuffer, 80, myFile)) > 0)
        {
            /* Cada vez que leo bien reseteo el contador de errores */
            readretry = READ_RETRY;
            /* Voy contando lo bytes leidos */
            bytecount += byteread;
            /* Salt�o los ':' iniciales */
            ConvertAsciiToHex(&asciiBuffer[1],hexRec);

            if( CheckHexRecord(hexRec))
            {
                validHex = 1;
                break;
            }
            // Blink LED
            BlinkLed(300);
        }//while(1)

        /* Me fijo si ley� todo */
        if(readretry && bytecount < myFile->size && validHex == 0)
        {
            readretry--;
            /* Cierro el archivo */
            FSfclose(myFile);
            /* Lo vuelvo a abrir */
            myFile = FSfopen(PROGRAM_FILE_NAME, "r");
            if( !myFile) myFile = FSfopen(PROGRAM_FILE_NAME_BK, "r");
            /* me paro donde se hab�a cortado */
            FSfseek(myFile, bytecount, SEEK_SET);
        }

    }

    /* Si el HEX no sirve */
    if(validHex == 0)
    {
        if(ValidAppPresent())
        {
            JumpToApp();
        }
        else
        {
            //Indicate error and stay in while loop.
            Error(3);
        }
    }

    /* Cierro el archivo */
    FSfclose(myFile);
    /* Lo vuelvo a abrir */
    myFile = FSfopen(PROGRAM_FILE_NAME, "r");
    if( !myFile) myFile = FSfopen(PROGRAM_FILE_NAME_BK, "r");
#endif /* VERIFY_PROGRAM */
    // Erase Flash (Block Erase the program Flash)
    EraseFlash();

    /* Un parche de reintentos para salvar errores de lectura de las SD */
    bytecount = 0;
    readretry = READ_RETRY;
    while(readretry && bytecount < myFile->size)
    {
        while((byteread = readLine(asciiBuffer, 80, myFile)) > 0)
        {
            /* Cada vez que leo bien reseteo el contador de errores */
            readretry = READ_RETRY;
            /* Voy contando lo bytes leidos */
            bytecount += byteread;
            /* Salt�o los ':' iniciales */
            ConvertAsciiToHex(&asciiBuffer[1],hexRec);

            if(WriteHexRecord2Flash(hexRec))
            {
                /* Cuando encuentra el registro final sale con 1 */
                mLED = 0;
                JumpToApp();
            }
            // Blink LED
            BlinkLed(300);
        }//while(1)

        /* Me fijo si ley� todo */
        if(readretry && bytecount < myFile->size)
        {
            readretry--;
            /* Cierro el archivo */
            FSfclose(myFile);
            /* Lo vuelvo a abrir */
            myFile = FSfopen(PROGRAM_FILE_NAME, "r");
            if( !myFile) myFile = FSfopen(PROGRAM_FILE_NAME_BK, "r");
            /* me paro donde se hab�a cortado */
            FSfseek(myFile, bytecount, SEEK_SET);
        }

    }
    /* Si pasa por ac� es un error en el archivo HEX */
    FSfclose(myFile);
    Error(4);
    return 0;
}

/********************************************************************
* Function: 	JumpToApp()
*
* Precondition: 
*
* Input: 		None.
*
* Output:		
*
* Side Effects:	No return from here.
*
* Overview: 	Jumps to application.
*
*			
* Note:		 	None.
********************************************************************/
void JumpToApp(void)
{
#ifdef ALLOW_WRITES
    int bkp = 0;
    char backup_name[TOTAL_FILE_SIZE+1];

    /* Reenombro el archivo del programa para no cargarlo de nuevo */
    strcpy(backup_name, PROGRAM_FILE_NAME);
    if(myFile)
    {
        while(bkp < 1000)
        {
            sprintf( (char*)(strchr(backup_name, '.')+1), "%03i", bkp );
            if(FSrename(backup_name, myFile) == 0) break;
            bkp++;
        }
        if(bkp == 1000)
        {
            FSfclose(myFile);
            FSremove(PROGRAM_FILE_NAME);
        }
    }
#endif /* ALLOW_WRITES */

    /* Antes de saltar a la aplicacion deben estar todas las interrupciones inhabilitadas */

    /* Salto */
    void (*fptr)(void);
    fptr = (void (*)(void))USER_APP_RESET_ADDRESS;
    fptr();
}	

/********************************************************************
* Function: 	ConvertAsciiToHex()
*
* Precondition: 
*
* Input: 		Ascii buffer and hex buffer.
*
* Output:		
*
* Side Effects:	No return from here.
*
* Overview: 	Converts ASCII to Hex.
*
*			
* Note:		 	None.
********************************************************************/
void ConvertAsciiToHex(UINT8* asciiRec, UINT8* hexRec)
{
	UINT8 i = 0;
	UINT8 k = 0;
	UINT8 hex;
	
	
	while((asciiRec[i] >= 0x30) && (asciiRec[i] <= 0x66))
	{
		// Check if the ascci values are in alpha numeric range.
		
		if(asciiRec[i] < 0x3A)
		{
			// Numerical reperesentation in ASCII found.
			hex = asciiRec[i] & 0x0F;
		}
		else
		{
			// Alphabetical value.
			hex = 0x09 + (asciiRec[i] & 0x0F);						
		}
	
		// Following logic converts 2 bytes of ASCII to 1 byte of hex.
		k = i%2;
		
		if(k)
		{
			hexRec[i/2] |= hex;
			
		}
		else
		{
			hexRec[i/2] = (hex << 4) & 0xF0;
		}	
		i++;		
	}		
	
}
// Do not change this
#define FLASH_PAGE_SIZE 0x1000
/********************************************************************
* Function: 	EraseFlash()
*
* Precondition: 
*
* Input: 		None.
*
* Output:		
*
* Side Effects:	No return from here.
*
* Overview: 	Erases Flash (Block Erase).
*
*			
* Note:		 	None.
********************************************************************/
void EraseFlash(void)
{
	void * pFlash;
    UINT result;
    INT i;

    pFlash = (void*)APP_FLASH_BASE_ADDRESS;									
    for( i = 0; i < ((APP_FLASH_END_ADDRESS - APP_FLASH_BASE_ADDRESS + 1)/FLASH_PAGE_SIZE); i++ )
    {
	     result = NVMemErasePage( pFlash + (i*FLASH_PAGE_SIZE) );
        // Assert on NV error. This must be caught during debug phase.

        if(result != 0)
        {
           // We have a problem. This must be caught during the debug phase.
            while(1);
        } 
        // Blink LED to indicate erase is in progress.
        BlinkLed(10);
    }			           	     
}

/********************************************************************
* Function: 	WriteHexRecord2Flash()
*
* Precondition: 
*
* Input: 		None.
*
* Output:		
*
* Side Effects:	No return from here.
*
* Overview: 	Writes Hex Records to Flash.
*
*			
* Note:		 	None.
********************************************************************/
int WriteHexRecord2Flash(UINT8* HexRecord)
{
    static T_HEX_RECORD HexRecordSt;
    UINT8 Checksum = 0;
    UINT8 i;
    UINT WrData;
    UINT RdData;
    void* ProgAddress;
    UINT result;

    HexRecordSt.RecDataLen = HexRecord[0];
    HexRecordSt.RecType = HexRecord[3];
    HexRecordSt.Data = &HexRecord[4];
	
    // Hex Record checksum check.
    for(i = 0; i < HexRecordSt.RecDataLen + 5; i++)
    {
            Checksum += HexRecord[i];
    }
	
    if(Checksum != 0)
    {
        //Error. Hex record Checksum mismatch.
        //Indicate Error by switching ON all LEDs.
        Error(5);
    }
    else
    {
        // Hex record checksum OK.
        switch(HexRecordSt.RecType)
        {
            case DATA_RECORD:  //Record Type 00, data record.
                HexRecordSt.Address.byte.MB = 0;
                HexRecordSt.Address.byte.UB = 0;
                HexRecordSt.Address.byte.HB = HexRecord[1];
                HexRecordSt.Address.byte.LB = HexRecord[2];
                // Derive the address.
                HexRecordSt.Address.Val = HexRecordSt.Address.Val + HexRecordSt.ExtLinAddress.Val + HexRecordSt.ExtSegAddress.Val;
                while(HexRecordSt.RecDataLen) // Loop till all bytes are done.
                {
                    // Convert the Physical address to Virtual address.
                    ProgAddress = (void *)PA_TO_KVA0(HexRecordSt.Address.Val);
                    // Make sure we are not writing boot area and device configuration bits.
                    if(((ProgAddress >= (void *)APP_FLASH_BASE_ADDRESS) && (ProgAddress <= (void *)APP_FLASH_END_ADDRESS))
                       && ((ProgAddress < (void*)DEV_CONFIG_REG_BASE_ADDRESS) || (ProgAddress > (void*)DEV_CONFIG_REG_END_ADDRESS)))
                    {
                        if(HexRecordSt.RecDataLen < 4)
                        {
                            // Sometimes record data length will not be in multiples of 4. Appending 0xFF will make sure that..
                            // we don't write junk data in such cases.
                            WrData = 0xFFFFFFFF;
                            memcpy(&WrData, HexRecordSt.Data, HexRecordSt.RecDataLen);
                        }
                        else
                        {
                            memcpy(&WrData, HexRecordSt.Data, 4);
                        }
                        // Write the data into flash.
                        result = NVMemWriteWord(ProgAddress, WrData);
                        // Assert on error. This must be caught during debug phase.
                        if(result != 0)
                        {
                            while(1);
                        }
                    }
                    // Increment the address.
                    HexRecordSt.Address.Val += 4;
                    // Increment the data pointer.
                    HexRecordSt.Data += 4;
                    // Decrement data len.
                    if(HexRecordSt.RecDataLen > 3)
                    {
                        HexRecordSt.RecDataLen -= 4;
                    }
                    else
                    {
                        HexRecordSt.RecDataLen = 0;
                    }
                }
                break;
        case EXT_SEG_ADRS_RECORD:  // Record Type 02, defines 4th to 19th bits of the data address.
            HexRecordSt.ExtSegAddress.byte.MB = 0;
            HexRecordSt.ExtSegAddress.byte.UB = HexRecordSt.Data[0];
            HexRecordSt.ExtSegAddress.byte.HB = HexRecordSt.Data[1];
            HexRecordSt.ExtSegAddress.byte.LB = 0;
            // Reset linear address.
            HexRecordSt.ExtLinAddress.Val = 0;
            break;
        case EXT_LIN_ADRS_RECORD:   // Record Type 04, defines 16th to 31st bits of the data address.
            HexRecordSt.ExtLinAddress.byte.MB = HexRecordSt.Data[0];
            HexRecordSt.ExtLinAddress.byte.UB = HexRecordSt.Data[1];
            HexRecordSt.ExtLinAddress.byte.HB = 0;
            HexRecordSt.ExtLinAddress.byte.LB = 0;
            // Reset segment address.
            HexRecordSt.ExtSegAddress.Val = 0;
            break;
        case END_OF_FILE_RECORD:  //Record Type 01, defines the end of file record.
            HexRecordSt.ExtSegAddress.Val = 0;
            HexRecordSt.ExtLinAddress.Val = 0;
            return 1; /* Listo para saltar a la APP */
            break;
        default:
            HexRecordSt.ExtSegAddress.Val = 0;
            HexRecordSt.ExtLinAddress.Val = 0;
            break;
        }
    }
    return 0;
}	

/********************************************************************
* Function: 	ValidAppPresent()
*
* Precondition: 
*
* Input: 		None.
*
* Output:		TRUE: If application is valid.
*
* Side Effects:	None.
*
* Overview: 	Logic: Check application vector has 
				some value other than "0xFFFFFF"
*
*			
* Note:		 	None.
********************************************************************/
BOOL ValidAppPresent(void)
{
	volatile UINT32 *AppPtr;
	
	AppPtr = (UINT32*)USER_APP_RESET_ADDRESS;

	if(*AppPtr == 0xFFFFFFFF)
	{
		return FALSE;
	}
	else
	{
		return TRUE;
	}
}			

/********************************************************************
* Function: 	readLine()
*
* Precondition:
*
* Input:
*
* Output:
*
* Side Effects:	None.
*
* Overview:
*
*
* Note:
********************************************************************/
int readLine(void *ptr, size_t max_len, FSFILE *stream)
{
    int len = 0;
    char *p;
    int rec_len;

    p = ptr;

    while(FSfread(p,1,1,stream))
    {
        if(*p != 0x0d && *p != 0x0a) break;
    }
    
    /* Verifico que sea el inicio de l�nea */
    if(*p != ':') return 0;
    /* Avanzo el puntero */
    p++;
    len++;
    /* Leo el tipo de registro */
    FSfread(p,1,2,stream);
    p[2] = 0;
    /* Segun el tipo de registro me cargo el tama�o */
    switch((int)strtol(p, NULL, 16))
    {
        case 0x00:
            rec_len = 11;
            break;
        case 0x02:
            rec_len = 15;
            break;
        case 0x04:
            rec_len = 19;
            break;
        case 0x08:
            rec_len = 27;
            break;
        case 0x0c:
            rec_len = 35;
            break;
        case 0x10:
            rec_len = 43;
            break;
        default:
            return 0;
    }
    /* Avanzo el puntero */
    p += 2;
    len += 2;
    /* le saco los 3 caracteres ya leidos leidos */
    rec_len -= 3;
    /* Le agrego uno para que quede adentro el linefeed */
    rec_len += 1;
    /* si es un tipo de registro conocido leo todo el registro de una */
    if(FSfread(p,1,rec_len,stream) != rec_len) return 0;
    p[rec_len] = 0x00;
    len += rec_len;
    return len;
}

/********************************************************************
 *
 ********************************************************************/
int CheckHexRecord(UINT8* HexRecord)
{
    UINT8 Checksum = 0;
    UINT8 RecDataLen;
    UINT8 i;

    // Hex Record checksum check.
    for(i = 0; i < RecDataLen + 5; i++)
    {
            Checksum += HexRecord[i];
    }

    if(Checksum != 0)
    {
        /* Error de checksum en un registro */
        return 0;
    }
    else
    {
        // Hex record checksum OK.
        switch(HexRecord[3])
        {
            case DATA_RECORD:  //Record Type 00, data record.
                break;
        case EXT_SEG_ADRS_RECORD:  // Record Type 02, defines 4th to 19th bits of the data address.
            break;
        case EXT_LIN_ADRS_RECORD:   // Record Type 04, defines 16th to 31st bits of the data address.
            break;
        case END_OF_FILE_RECORD:  //Record Type 01, defines the end of file record.
            return 1; /* Listo para saltar a la APP */
            break;
        default:
            break;
        }
    }
    /* Le falta el record final */
    return 0;
}
