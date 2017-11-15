/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board GPIO driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "gpio-board.h"

static GpioIrqHandler *GpioIrq[16];
static void LOWAPP_EXTI_Callback( uint16_t gpioPin );
static void LOWAPP_EXTI_IRQHandler( uint16_t gpioPin );

//Done, Not tested
void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if( pin == NC )
    {
        return;
    }
    obj->pin = pin;		//pin is port and number defined in pinName-board exp. PA12
    obj->pinIndex = ( 0x01 << ( obj->pin & 0x0F ) );	//pinIndex is pin number exp. GPIO_Pin_12

    //Save pin's port to obj->port exp. PORTA
    //Enable clock for selected port
    if( ( obj->pin & 0xF0 ) == 0x00 )
    {
        obj->port = GPIOA;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    }
    else if( ( obj->pin & 0xF0 ) == 0x10 )
    {
    	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    }
    else if( ( obj->pin & 0xF0 ) == 0x20 )
    {
    	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    }
    else if( ( obj->pin & 0xF0 ) == 0x30 )
    {
    	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    }
    else if( ( obj->pin & 0xF0 ) == 0x40 )
    {
        obj->port = GPIOE;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    }
    else
    {
    	//STM32F103VE has ports A to E
    	return;
    }

    GPIO_InitStructure.Pin =  obj->pinIndex ;
    GPIO_InitStructure.Speed = GPIO_Speed_10MHz;

    //According to STM32F013 Ref manual, pull up/down resistor can be enabled
    //only in digital input mode
    if( mode == PIN_INPUT )	//Digital input
    {
    	if(type == PIN_PULL_UP)
    		GPIO_InitStructure.Mode = GPIO_Mode_IPU;
    	else if (type == PIN_PULL_DOWN)
    		GPIO_IntiStructure.Mode = GPIO_Mode_IPD;
    	else
    		GPIO_InitStructure.Mode = GPIO_Mode_IN_FLOATING;
    }
    else if( mode == PIN_ANALOGIC )		 //Analog function
    {
        GPIO_InitStructure.Mode = GPIO_Mode_AIN;
    }
    else if( mode == PIN_ALTERNATE_FCT ) //Alternate function
    {
        if( config == PIN_OPEN_DRAIN )
        {
            GPIO_InitStructure.Mode = GPIO_Mode_AF_OD;
        }
        else
        {
            GPIO_InitStructure.Mode = GPIO_Mode_AF_PP;
        }
        GPIO_InitStructure.Alternate = value;
    }
    else // Output mode
    {
        if( config == PIN_OPEN_DRAIN )
        {
            GPIO_InitStructure.Mode = GPIO_Mode_Out_OD;
        }
        else
        {
            GPIO_InitStructure.Mode = GPIO_Mode_Out_PP;
        }
    }

    GPIO_Init( obj->port, &GPIO_InitStructure );

    // Sets initial output value
    if( mode == PIN_OUTPUT )
    {
        GpioMcuWrite( obj, value );
    }
}
//Ported, but not tested jet
//This function must be called after GpioMcuInit() function for the same pin
//User must handle NVIC init and irq priorites manually in vsndriversconf.c
void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    uint32_t priority = 0;
    IRQn_Type IRQnb = EXTI0_IRQn;
    EXTI_InitTypeDef EXTI_InitStructure;

    if( irqHandler == NULL )
    {
        return;
    }

    EXTI_InitStructure.EXTI_Line = obj->pinIndex;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

    if( irqMode == IRQ_RISING_EDGE )
    {
    	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    }
    else if( irqMode == IRQ_FALLING_EDGE )
    {
    	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    }
    else
    {
    	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    }

    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearITPendingBit(obj->pinIndex);
    GPIO_EXTILineConfig(obj->port, obj->pinIndex);

    GpioIrq[(obj->pin ) & 0x0F] = irqHandler;

}
//Ported but not tested
void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    GPIO_InitStructure.Pin =  obj->pinIndex ;
    GPIO_InitStructure.Mode = GPIO_Mode_AIN;
    GPIO_StructInit( obj->port, &GPIO_InitStructure );
}
//Ported but not tested
void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
    if( ( obj == NULL ) || ( obj->port == NULL ) )
    {
        assert_param( FAIL );
    }
    // Check if pin is not connected
    if( obj->pin == NC )
    {
        return;
    }
    if(value)
    	GPIO_WriteBit(obj->port, obj->pinIndex, 1);
    else
    	GPIO_WriteBit(obj->port, obj->pinIndex, 0);
}
//Ported but not tested
void GpioMcuToggle( Gpio_t *obj )
{
    if( ( obj == NULL ) || ( obj->port == NULL ) )
    {
        assert_param( FAIL );
    }

    // Check if pin is not connected
    if( obj->pin == NC )
    {
        return;
    }
    //Toggle pin
    if(GPIO_ReadOutputDataBit( obj->port, obj->pinIndex ))
    	GPIO_ResetBits(obj->port, obj->pinIndex);
    else
    	GPIO_SetBits(obj->port, obj->pinIndex);
}
//Ported but not tested
uint32_t GpioMcuRead( Gpio_t *obj )
{
    if( obj == NULL )
    {
        assert_param( FAIL );
    }
    // Check if pin is not connected
    if( obj->pin == NC )
    {
        return 0;
    }
    return GPIO_ReadInputDataBit( obj->port, obj->pinIndex );
}
//Ported but not tested
void EXTI0_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_0 );
}
//Ported but not tested
void EXTI1_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_1 );
}
//Ported but not tested
void EXTI2_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_2 );
}
//Ported but not tested
void EXTI3_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_3 );
}
//Ported but not tested
void EXTI4_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_4 );
}
//Ported but not tested
void EXTI9_5_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_5 );
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_6 );
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_7 );
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_8 );
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_9 );
}
//Ported but not tested
void EXTI15_10_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_10 );
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_11 );
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_12 );
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_13 );
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_14 );
    LOWAPP_EXTI_IRQHandler( GPIO_Pin_15 );
}
//Ported but not tested
static void LOWAPP_EXTI_IRQHandler(uint16_t gpioPin)
{
  /* EXTI line interrupt detected */
	if(EXTI_GetITStatus(gpioPin) != RESET)
	{
		EXTI_ClearITPendingBit(gpioPin);
		LOWAPP_EXTI_Callback(gpioPin);
	}
}
//Ported but not tested
static void LOWAPP_EXTI_Callback( uint16_t gpioPin )
{
    uint8_t callbackIndex = 0;

    if( gpioPin > 0 )
    {
        while( gpioPin != 0x01 )
        {
            gpioPin = gpioPin >> 1;
            callbackIndex++;
        }
    }

    if( GpioIrq[callbackIndex] != NULL )
    {
        GpioIrq[callbackIndex]( );
    }
}
