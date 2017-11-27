/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board SPI driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "spi-board.h"
#include "stm32f10x_spi.h"
#include "vsnspi_new.h"

/*SPI is currently hardcoded to J2 connector. Spi2 connects to J2 through U8 and U1 analog
switches.
Function	J2-Pin		MCU-Pin		Switch-Control
MOSI		PD10		PB15		PC8
MISO		PD12		PB14		PC8
SCK			PD11		PB13		PC8
NSS			PD1			PD1			/

*/

static void vsnSPIerrorcallBackMain (void* callBackStructOfPeripheral){
	vsnSPI_CommonStructure* x;
	x = (vsnSPI_CommonStructure*) callBackStructOfPeripheral;
	vsnSPI_chipSelect(x, SPI_CS_HIGH);
	printf("ERROR: SPIerrorCallBack was execute!!!\n");
}

void SpiInit( Spi_t *obj, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
	SPI_InitTypeDef SPI_InitStructure;
	vsnSPI_ErrorStatus statusSpi;

	//If SPI SWICTH pin is defined than pins on connector are connected to MCU through analog switch.
	//So we need to intialize mosi, miso and sclk as floating inputs
	/* Configure  */
#ifdef SPI_SWITCH
	GpioInit( &obj->Mosi, mosi, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	GpioInit( &obj->Miso, miso, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	GpioInit( &obj->Sclk, sclk, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	//Define SPI_Switch pin as output pin and set it to one so you make connection between
	//connector and MCU analog switch
	GpioInit( &obj->Switch, SPI_SWITCH, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
#endif
	//Initalize NSS as output and set it to 1 to disable slave
	GpioInit(&obj->Nss, nss, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
	/* Configure hardware */
	vsnSPI_initHW(SPI_PORT);

	/* Configure SPI init structure*/
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//OK
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	//OK
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//OK
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//OK
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//OK
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//OK
	SPI_InitStructure.SPI_CRCPolynomial = 0;	//OK

	//Initialize SPI Common Structure
    statusSpi = vsnSPI_initCommonStructure(&obj->Spi, SPI_PORT, &SPI_InitStructure, obj->Nss.pinIndex, obj->Nss.port,10000000);
    //Initalize SPI
    statusSpi = vsnSPI_Init(&obj->Spi, vsnSPIerrorcallBackMain);
}

void SpiDeInit( Spi_t *obj )
{
	vsnSPI_deInit( &obj->Spi );
#ifdef SPI_SWITCH
	//Disable analog switch
	GpioInit( &obj->Switch, SPI_SWITCH, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
#endif
    GpioInit( &obj->Mosi, obj->Mosi.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Miso, obj->Miso.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );
    GpioInit( &obj->Sclk, obj->Sclk.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Nss, obj->Nss.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );

}
//This function is no longer needed
void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave )
{
    while(1){};
}
//This function is no longer needed
void SpiFrequency( Spi_t *obj, uint32_t hz )
{
    while(1){};
}


FlagStatus SpiGetFlag( Spi_t *obj, uint16_t flag )
{
    FlagStatus bitstatus = RESET;
		if( ( obj->Spi.devSPIx->SR & flag ) != ( uint16_t )RESET )
		{
		// SPI_I2S_FLAG is set
		bitstatus = SET;
		}
		else
		{
		// SPI_I2S_FLAG is reset
		bitstatus = RESET;
		}
    // Return the SPI_I2S_FLAG status
    return  bitstatus;
}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
    uint8_t rxData = 0;

    if(  (obj == NULL) || (obj->Spi.devSPIx == NULL))
    {
        assert_param( FAIL );
    }
    //Send data
    while( SpiGetFlag( obj, SPI_I2S_FLAG_TXE ) == RESET );
    obj->Spi.devSPIx->DR = ( uint16_t ) ( outData & 0xFF );
    //Read data
    while( SpiGetFlag( obj, SPI_I2S_FLAG_RXNE ) == RESET );
    rxData = ( uint16_t ) obj->Spi.devSPIx->DR;

    return( rxData );
}

