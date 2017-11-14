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

void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    uint32_t priority = 0;

    IRQn_Type IRQnb = EXTI0_IRQn;
    GPIO_InitTypeDef   GPIO_InitStructure;

    if( irqHandler == NULL )
    {
        return;
    }

    GPIO_InitStructure.Pin =  obj->pinIndex;

    if( irqMode == IRQ_RISING_EDGE )
    {
        GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    }
    else if( irqMode == IRQ_FALLING_EDGE )
    {
        GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
    }
    else
    {
        GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
    }

    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init( obj->port, &GPIO_InitStructure );

    switch( irqPriority )
    {
    case IRQ_VERY_LOW_PRIORITY:
    case IRQ_LOW_PRIORITY:
        priority = 3;
        break;
    case IRQ_MEDIUM_PRIORITY:
        priority = 2;
        break;
    case IRQ_HIGH_PRIORITY:
        priority = 1;
        break;
    case IRQ_VERY_HIGH_PRIORITY:
    default:
        priority = 0;
        break;
    }

    switch( obj->pinIndex )
    {
    case GPIO_PIN_0:
        IRQnb = EXTI0_IRQn;
        break;
    case GPIO_PIN_1:
        IRQnb = EXTI1_IRQn;
        break;
    case GPIO_PIN_2:
        IRQnb = EXTI2_IRQn;
        break;
    case GPIO_PIN_3:
        IRQnb = EXTI3_IRQn;
        break;
    case GPIO_PIN_4:
        IRQnb = EXTI4_IRQn;
        break;
    case GPIO_PIN_5:
    case GPIO_PIN_6:
    case GPIO_PIN_7:
    case GPIO_PIN_8:
    case GPIO_PIN_9:
        IRQnb = EXTI9_5_IRQn;
        break;
    case GPIO_PIN_10:
    case GPIO_PIN_11:
    case GPIO_PIN_12:
    case GPIO_PIN_13:
    case GPIO_PIN_14:
    case GPIO_PIN_15:
        IRQnb = EXTI15_10_IRQn;
        break;
    default:
        break;
    }

    GpioIrq[(obj->pin ) & 0x0F] = irqHandler;

    HAL_NVIC_SetPriority( IRQnb , priority, 0 );
    HAL_NVIC_EnableIRQ( IRQnb );
}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    GPIO_InitStructure.Pin =  obj->pinIndex ;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init( obj->port, &GPIO_InitStructure );
}

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
    HAL_GPIO_TogglePin( obj->port, obj->pinIndex );
}

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
    return HAL_GPIO_ReadPin( obj->port, obj->pinIndex );
}

void EXTI0_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_0 );
}

void EXTI1_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_1 );
}

void EXTI2_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_2 );
}

void EXTI3_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_3 );
}

void EXTI4_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_4 );
}

void EXTI9_5_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_5 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_6 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_7 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_8 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_9 );
}

void EXTI15_10_IRQHandler( void )
{
#if !defined( USE_NO_TIMER )
    RtcRecoverMcuStatus( );
#endif
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_10 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_11 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_12 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_13 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_14 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_15 );
}

void HAL_GPIO_EXTI_Callback( uint16_t gpioPin )
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
