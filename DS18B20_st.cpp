
#include "DS18B20_uART2.hpp"
/* Includes */
#include <stddef.h>
#include <stm32l431xx.h>
#include "stdint.h"


/********************************************************************************/
/**
 * @brief   AF8 PC12 pin masks
 */


/**
 * @brief   Reset command
 */
#define DS18B20_RESET_CMD                    ((uint8_t) 0xF0)

/**
 * @brief   Logical bit values
 */
#define BIT_0                                ((uint8_t) 0x00)
#define BIT_1                                ((uint8_t) 0xFF)

/**
 * @brief   Conversion time in ms, from DS18B20 datasheet
 */
#define MAX_CONVERSION_TIME                  ((uint32_t) 750)



/**
 * @brief   Temperature convert, {Skip ROM = 0xCC, Convert = 0x44}
 */
static const uint8_t temp_convert[] =
        {
                BIT_0, BIT_0, BIT_1, BIT_1, BIT_0, BIT_0, BIT_1, BIT_1,
                BIT_0, BIT_0, BIT_1, BIT_0, BIT_0, BIT_0, BIT_1, BIT_0
        };

/**
 * @brief   Temperature data read, {Skip ROM = 0xCC, Scratch read = 0xBE}
 */
static const uint8_t temp_read[] =
        {
                BIT_0, BIT_0, BIT_1, BIT_1, BIT_0, BIT_0, BIT_1, BIT_1,
                BIT_0, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_0, BIT_1,
                BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1,
                BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1
        };

/**
 * @}
 */

/**
 * @defgroup DS18B20_private_variables
 * @{
 */

/**
 * @brief   Received temperature data using DMA
 */
static uint8_t temperatureData[sizeof(temp_read)];

/**
 * @brief   Temperature data received flag
 */
static uint8_t temperatureDataReceived = 0;

/**
 * @brief   Current temperature value in degree celsius
 */
static float currentTemperature = 0;

/**
 * @}
 */

/**
 * @defgroup DS18B20_private_function_prototypes
 * @{
 */

/**
 * @brief   DMA command transmit
 * @note
 * @param   cmd, size
 * @retval  None
 */
static void cmdTransmit(const uint8_t * cmd, uint8_t size,USART_TypeDef *usart);

/**
 * @brief   DMA command receive
 * @note
 * @param   cmd, size
 * @retval  None
 */
static void cmdReceive(const uint8_t * cmd, uint8_t size,USART_TypeDef *usart);

/**
 * @brief   Send reset pulse to DS18B20
 * @note
 * @param   None
 * @retval  None
 */
static uint8_t cmdReset(USART_TypeDef *usart);

/**
 * @}
 */

/**
 * @defgroup DS18B20_private_functions
 * @{
 */

/**
 * @brief   DMA string transmit
 * @note    IMPORTANT: Since we send and receive the reset pulse without DMA,
 *          its necessary to clear any pending DMA requests before
 *          enable transmission DMA stream.
 * @param   cmd, size
 * @retval  None
 */
static void cmdTransmit(const uint8_t * cmd, uint8_t size,USART_TypeDef *usart)
{
  DMA_Channel_TypeDef *pDMASt;  ///Stream


  /////**   USART1 Tx  DMA stream4 channel2**/////
    /////**   USART2 Tx  DMA stream7 channel2**/////
    if (usart==USART1)
    {pDMASt=DMA1_Channel4;}
    else if (usart==USART2)
    {pDMASt=DMA1_Channel7;}

    /* Check null pointers */
    if(nullptr != cmd)
    {
        /* Wait until DMA1 stream 7is disabled */
        while(DMA_CCR_EN == (DMA_CCR_EN & pDMASt->CCR))
        {
            /* Do nothing, the enable flag shall reset
             * when DMA transfer complete */
        }

        /* Set memory address */
        pDMASt->CMAR = (uint32_t)cmd;

        /* Set number of data items */
        pDMASt->CNDTR = size;
        if (usart==USART2)
        {
        /* Clear all interrupt flags */
        DMA1->IFCR = (DMA_IFCR_CGIF7| DMA_IFCR_CHTIF7 | DMA_IFCR_CTCIF7
                       | DMA_IFCR_CTEIF7);}
        else if (usart==USART1)
        {
            /* Clear all interrupt flags */
            DMA1->IFCR = (DMA_IFCR_CGIF4| DMA_IFCR_CHTIF4 | DMA_IFCR_CTCIF4
                          | DMA_IFCR_CTEIF4);
        }

        /* Clear any UART pending DMA requests */
        usart->CR3 &= ~USART_CR3_DMAT;

        /* Enable DMA mode for transmitter */
        usart->CR3 |= USART_CR3_DMAT;

        /* Enable DMA 1 stream */
        pDMASt->CCR |= DMA_CCR_EN;
    }
    else
    {
        /* Null pointers, do nothing */
    }
}

/**
 * @brief   DMA string receive
 * @note    IMPORTANT: Since we send and receive the reset pulse without DMA,
 *          its necessary to clear any pending DMA requests before
 *          enable reception DMA stream.
 * @param   cmd, size
 * @retval  None
 */
static void cmdReceive(const uint8_t * cmd, uint8_t size,USART_TypeDef *usart)
{
    DMA_Channel_TypeDef *pDMASt;  ///Stream

    if (usart==USART1){pDMASt=DMA1_Channel5;}
    else if (usart==USART2){pDMASt=DMA1_Channel6;}
    /* Check null pointers */
    if(nullptr != cmd)
    {
        /* Wait until DMA1 stream 0 is disabled */
        while(DMA_CCR_EN == (DMA_CCR_EN & pDMASt->CCR))
        {
            /* Do nothing, the enable flag shall reset
             * when DMA transfer complete */
        }

        /* Set memory address */
        pDMASt->CMAR = (uint32_t)cmd;

        /* Set number of data items */
        pDMASt->CNDTR = size;

        if (usart==USART2)
        {
            /* Clear all interrupt flags */
            DMA1->IFCR = (DMA_IFCR_CGIF6| DMA_IFCR_CHTIF6 | DMA_IFCR_CTCIF6
                          | DMA_IFCR_CTEIF6);}
        else if (usart==USART1)
        {
            /* Clear all interrupt flags */
            DMA1->IFCR = (DMA_IFCR_CGIF5| DMA_IFCR_CHTIF5 | DMA_IFCR_CTCIF5
                          | DMA_IFCR_CTEIF5);
        }

        /* Clear any UART pending DMA requests */
        usart->CR3 &= ~USART_CR3_DMAR;

        /* Enable DMA mode for reception */
        usart->CR3 |= USART_CR3_DMAR;

        /* Enable DMA 1 stream 0 */
        pDMASt->CCR |= DMA_CCR_EN;
    }
    else
    {
        /* Null pointers, do nothing */
    }
}

/**
 * @brief   Send reset pulse to DS18B20
 * @note
 * @param   None
 * @retval  None
 */
static uint8_t cmdReset(USART_TypeDef *usart)
{
    uint8_t isSensorDetected = 0;
  //// Set Baudrate function
    /* Disable UART5 prescaler and outputs */
    usart->CR1 &= ~USART_CR1_UE;

    /* Set baud rate = 9600 Bps
     * USARTDIV = Fck / (16 * baud_rate)
     *          = 45000000 / (16 * 9600) = 292.96
     *
     * DIV_Fraction = 16 * 0.96 = 15.36 = 15 = 0xF
     * DIV_Mantissa = 292 = 0x124
     *
     * BRR          = 0x124F */
    usart->BRR = 0x124F;// set baudrate to 115200

    /* Enable UART5 prescaler and outputs */
    usart->CR1 |= USART_CR1_UE;

    /* Check USART status register */
    while(!(usart->ISR & USART_ISR_TXE))
    {
        /* Wait for transmission buffer empty flag */
    }

    /* Write reset command */
    usart->TDR = DS18B20_RESET_CMD;

    /* Check USART status register */
    while(!(usart->ISR & USART_ISR_TC))
    {
        /* Wait for transmission complete flag */
    }

    /* Read Rx Data */
    uint16_t Rx = usart->TDR;

    /* Check sensor presence */
    if((DS18B20_RESET_CMD != Rx) && ( BIT_0 != Rx))
    {
        /* Temp sensor was detected */
        isSensorDetected = 1;
    }
    else
    {
        /* Do nothing, No sensor was detected */
    }

    /* Disable UART5 prescaler and outputs */
    usart->CR1 &= ~USART_CR1_UE;

    /* Set baud rate = 115200 Bps
     * USARTDIV = Fck / (16 * baud_rate)
     *          = 45000000 / (16 * 115200) = 24.41
     *
     * DIV_Fraction = 16 * 0.41 = 6.56 = 7 = 0x7
     * DIV_Mantissa = 24 = 0x18
     *
     * BRR          = 0x187 */
    usart->BRR = 0x187; // set baudrate to 115200

    /* Enable UART5 prescaler and outputs */
    usart->CR1 |= USART_CR1_UE;

    return isSensorDetected;
}

/**
 * @}
 */

/**
 * @defgroup DS18B20_exported_functions
 * @{
 */

/**
 * @brief   Configure GPIO
 * @note    UART5_TX -> PC12, UART5_RX -> PD2 (Not Used)
 *          UART5 mapped to alternate function AF8
 *          UART5 connected to APB1 with 45MHz max clock
 * @param   None
 * @retval  None
 */
void DS18B20::DS18B20_GPIO_Init(USART_TypeDef *usart)
{

    if (usart==USART2){
    /* Enable port A clock */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    /* Select alternate function mode */
    GPIOC->MODER &= ~(GPIO_MODER_MODER2);
    GPIOC->MODER |= GPIO_MODER_MODER2_1;

    /* Select output type open-drain */
    GPIOC->OTYPER |= GPIO_OTYPER_OT_2;

    /* Select output speed medium */
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR2);
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2_0;

    /* Select no pull-up, pull-down */
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR2);

    /* Select AF7 */
    GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL2);
    GPIOC->AFR[0] |= (0x7UL<<GPIO_AFRL_AFSEL2_Pos);}
    else if  (usart==USART1){
            /* Enable port A clock */
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

            /* Select alternate function mode */
            GPIOC->MODER &= ~(GPIO_MODER_MODER9);
            GPIOC->MODER |= GPIO_MODER_MODER9_1;

            /* Select output type open-drain */
            GPIOC->OTYPER |= GPIO_OTYPER_OT_9;

            /* Select output speed medium */
            GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR9);
            GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_0;

            /* Select no pull-up, pull-down */
            GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9);

            /* Select AF7 */
            GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL9);
            GPIOC->AFR[1] |= (0x7UL<<GPIO_AFRH_AFSEL9_Pos);}
}

/**
 * @brief   Configure DMA for UART TX
 * @note    UART5_TX -> DMA1_Stream7 (Channel 4)
 * @param   None
 * @retval  None
 */
void DS18B20:: DS18B20_TX_DMA_Init(USART_TypeDef *usart)
{DMA_Request_TypeDef *pDMAch;  /// channel
    DMA_Channel_TypeDef *pDMASt;  ///Stream
    /* Enable DMA clock in RCC */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
if(usart==USART1)
{pDMASt=DMA1_Channel5;}
else if(usart==USART2)
{pDMASt=DMA1_Channel7;}
    /* Make sure that the DMA1 stream  is disabled */
    if(DMA_CCR_EN == (DMA_CCR_EN & pDMASt->CCR))
    {
        /* DMA 1 stream  is enabled, shall be disabled first */
        pDMASt->CCR &= ~DMA_CCR_EN;

        /* Wait until EN bit is cleared */
        while(DMA_CCR_EN == (DMA_CCR_EN & pDMASt->CCR))
        {
            /* Do nothing until EN bit is cleared */
        }
    }
    else
    {
        /* Do nothing, stream 7 is not enabled */
    }

    /* Select the DMA channel 2 in  the CSELR */
    pDMAch->CSELR &= ~(DMA_CSELR_C2S);
    pDMAch->CSELR |= DMA_CSELR_C2S;

    /* Select stream priority very high */
    pDMASt->CCR |= DMA_CCR_PL;

    /* Select the data transfer direction memory-to-peripheral */
    pDMASt->CCR &= ~DMA_CCR_DIR;
    pDMASt->CCR |= DMA_CCR_DIR;

    /* Select memory and peripherals sizes byte (8-bit) */
    pDMASt->CCR &= ~DMA_CCR_MSIZE;
    pDMASt->CCR &= ~DMA_CCR_PSIZE;



    /* Select memory incremented mode, peripheral shall has fixed address */
    pDMASt->CCR |= DMA_CCR_MINC;

    /* Enable DMA transfer complete interrupt */
    pDMASt->CCR |= DMA_CCR_TCIE;

    /* Set peripheral address */
    pDMASt->CPAR = (uint32_t)&usart->TDR;
}

/**
 * @brief   Configure DMA for UART RX
 * @note    UART5_RX -> DMA1_Stream0 (Channel 4)
 * @param   None
 * @retval  None
 */
void  DS18B20::DS18B20_RX_DMA_Init(USART_TypeDef*usart)
{DMA_Request_TypeDef *pDMAch;  /// channel
    DMA_Channel_TypeDef *pDMASt;  ///Stream
    /* Enable DMA clock in RCC */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    if(usart==USART1)
    {pDMASt=DMA1_Channel4;}
    else if(usart==USART2)
    {pDMASt=DMA1_Channel6;}
    /* Enable DMA clock in RCC */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    /* Make sure that the DMA1 stream  is disabled */
    if(DMA_CCR_EN == (DMA_CCR_EN & pDMASt->CCR))
    {
        /* DMA 1 stream  is enabled, shall be disabled first */
        pDMASt->CCR &= ~DMA_CCR_EN;

        /* Wait until EN bit is cleared */
        while(DMA_CCR_EN == (DMA_CCR_EN & pDMASt->CCR))
        {
            /* Do nothing until EN bit is cleared */
        }
    }
    else
    {
        /* Do nothing, stream 5 is not enabled */
    }

    /* Select the DMA channel 2 in  the CSELR */
    pDMAch->CSELR &= ~(DMA_CSELR_C2S);
    pDMAch->CSELR |= DMA_CSELR_C2S;

    /* Select stream priority very high */
    pDMASt->CCR |= DMA_CCR_PL;

    /* Select the data transfer direction peripheral-to-memory */
    pDMASt->CCR &= ~DMA_CCR_DIR;

    /* Select memory and peripherals sizes byte (8-bit) */
    pDMASt->CCR &= ~DMA_CCR_MSIZE;
    pDMASt->CCR &= ~DMA_CCR_PSIZE;



    /* Select memory incremented mode, peripheral shall has fixed address */
    pDMASt->CCR |= DMA_CCR_MINC;

    /* Enable DMA transfer complete interrupt */
    pDMASt->CCR |= DMA_CCR_TCIE;

    /* Set peripheral address */
    pDMASt->CPAR = (uint32_t)&usart->RDR;
}

/**
 * @brief   Configure UART5 for DS18B20
 * @note
 * @param   None
 * @retval  None
 */
void DS18B20::DS18B20_UART5_Init(USART_TypeDef *usart)
{
    /* Enable UART5 clock */
    if(usart==USART2){RCC->APB1ENR1 = RCC_APB1ENR1_USART2EN;}
    else if(usart==USART1){RCC->APB2ENR = RCC_APB2ENR_USART1EN;}


    /* Select oversampling by 16 mode */
    usart->CR1 &= ~USART_CR1_OVER8;

    /* Select 1 Start bit, 8 Data bits, n Stop bit */
    usart->CR1 &= ~USART_CR1_M;

    /* Select 1 stop bit */
    usart->CR2 &= ~USART_CR2_STOP;

    /* Select three sample bit method */
    usart->CR3 &= ~USART_CR3_ONEBIT;

    /* Select Single-wire Half-duplex mode */
    usart->CR3 |= USART_CR3_HDSEL;
}

/**
 * @brief   Enable communications with DS18B20
 * @note
 * @param   None
 * @retval  None
 */
void DS18B20:: DS18B20_UART5_Enable(USART_TypeDef *usart)
{
    /* Enable UART5 */
    usart->CR1 |= USART_CR1_UE;

    /* Enable transmitter */
    usart->CR1 |= USART_CR1_TE;

    /* Enable receiver */
    usart->CR1 |= USART_CR1_RE;
}

/**
 * @brief   DS18B20 process function
 * @note
 * @param   None
 * @retval  None
 */
void DS18B20:: DS18B20_Process(USART_TypeDef *usart)
{
    /* Sensor detected flag */
    uint8_t isSensorDetected = 0;

    /* Send reset pulse */
    isSensorDetected = cmdReset(usart);

    /* Check if the sensor was detected */
    if(1 == isSensorDetected)
    {
        /* Turn on   LED */


        /* Send temperature conversion command */
        cmdTransmit(temp_convert, sizeof(temp_convert),usart);

        /* Wait conversion time */
       //Delay(MAX_CONVERSION_TIME);

        /* Send reset pulse */
        cmdReset(usart);

        /* Enable temperature data reception with DMA */
        cmdReceive(temperatureData, sizeof(temperatureData),usart);

        /* Send temperature read command */
        cmdTransmit(temp_read, sizeof(temp_read),usart);

        /* Check temperature data received flag */
        while (temperatureDataReceived == 0)
        {
            /* Wait until DMA receive temperature data */
        }

        /* Reset temperature data received flag */
        temperatureDataReceived = 0;

        /* Temporarily variable for extracting temperature data */
        uint16_t temperature = 0;

        /* Extract new temperature data */
        for (int idx = 16; idx < 32; idx++)
        {
            if (BIT_1 == temperatureData[idx])
            {
                /* Bit value is 1 */
                temperature = (temperature >> 1) | 0x8000;
            }
            else
            {
                /* Bit value is 0 */
                temperature = temperature >> 1;
            }
        }

        /* Copying new temperature data and divide by 16 for fraction part */
        currentTemperature = (float) temperature / (float) 16;

    }
    else
    {
        /* Turn   LED, indicates sensor detection failed */

        /* Temperature data not valid */
        currentTemperature = 0;
    }
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void DS18B20::UART5_TX_DMA_IRQ_Callback(void)
{
    /* Check transfer complete flag */
    if(DMA_ISR_TCIF1 & DMA1->ISR)
    {

        /* Clear all interrupt flags */
        DMA1->IFCR = (DMA_ISR_GIF4| DMA_ISR_TEIF4 | DMA_ISR_TCIF4
                      | DMA_ISR_HTIF4);
    }

    else if (DMA_ISR_TCIF7 & DMA1->ISR)
    {

        /* Clear all interrupt flags */
        DMA1->IFCR = (DMA_ISR_GIF7| DMA_ISR_TEIF7 | DMA_ISR_TCIF7
                      | DMA_ISR_HTIF7);
    }
    else
    {
        /* Do nothing, this interrupt is not handled */
    }
}




/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */

void DS18B20:: UART5_RX_DMA_IRQ_Callback(void)
{
    /* Check transfer complete flag */
    if(DMA_ISR_TCIF5 & DMA1->ISR)
    {
        /* Set transfer complete flag */
        temperatureDataReceived = 1;
        /* Clear all interrupt flags */
        DMA1->IFCR = (DMA_ISR_GIF5| DMA_ISR_TEIF5 | DMA_ISR_TCIF5
                      | DMA_ISR_HTIF5);
    }

    else if (DMA_ISR_TCIF6 & DMA1->ISR)
    {
        /* Set transfer complete flag */
        temperatureDataReceived = 1;
        /* Clear all interrupt flags */
        DMA1->IFCR = (DMA_ISR_GIF6| DMA_ISR_TEIF6 | DMA_ISR_TCIF6
                      | DMA_ISR_HTIF6);
    }
    else
    {
        /* Do nothing, this interrupt is not handled */
    }
}

/**
 * @}
 */
/**
 * @}
 */
/**
 * @}
 */
