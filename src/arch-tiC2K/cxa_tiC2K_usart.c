/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 * @author Christopher Armenio
 */
#include "cxa_tiC2K_usart.h"


// ******** includes ********
#include <stdbool.h>
#include <cxa_assert.h>
#include <cxa_delay.h>
#include <cxa_numberUtils.h>
#include <cxa_runLoop.h>

#include <cxa_tiC2K_gpio.h>
#include "Externals.h"


#define CXA_LOG_LEVEL		CXA_LOG_LEVEL_TRACE
#include <cxa_logger_implementation.h>


// ******** local macro definitions ********


// ******** local type definitions ********


// ******** local function prototypes ********
static cxa_ioStream_readStatus_t ioStream_cb_readByte_SCIA(uint8_t *const byteOut, void *const userVarIn);
static bool ioStream_cb_writeBytes_SCIA(void* buffIn, size_t bufferSize_bytesIn, void *const userVarIn);

static cxa_ioStream_readStatus_t ioStream_cb_readByte_SCIB(uint8_t *const byteOut, void *const userVarIn);
static bool ioStream_cb_writeBytes_SCIB(void* buffIn, size_t bufferSize_bytesIn, void *const userVarIn);

static void runLoopCb_fifoStatusPrint(void* userVarIn);

//__interrupt void scibTxISR(void);
//__interrupt void scibRxISR(void);
// ********  local variable declarations *********


// ******** global function implementations ********
void cxa_tiC2K_usart_init_noHH(cxa_tiC2K_usart_t *const usartIn, const uint32_t baudRate_bpsIn,
                               const uint32_t txPinConfigIn, const uint32_t txPinIn,
                               const uint32_t rxPinConfigIn, const uint32_t rxPinIn)
//void cxa_esp32_usart_init_noHH(cxa_esp32_usart_t *const usartIn, uart_port_t uartIdIn, const uint32_t baudRate_bpsIn,
//                          const gpio_num_t txPinIn, const gpio_num_t rxPinIn)
{
	cxa_assert(usartIn);

	// Setup TX Pin
	cxa_tiC2K_gpio_t usartTxGpio;
	cxa_tiC2K_gpio_init_output(&usartTxGpio, txPinConfigIn, txPinIn, GPIO_CORE_CPU1, GPIO_PIN_TYPE_STD, CXA_GPIO_POLARITY_NONINVERTED, 1);
	GPIO_setQualificationMode(txPinIn, GPIO_QUAL_ASYNC);

	// Setup RX Pin
	cxa_tiC2K_gpio_t usartRxGpio;
	cxa_tiC2K_gpio_init_input(&usartRxGpio, rxPinConfigIn, rxPinIn, GPIO_CORE_CPU1, GPIO_PIN_TYPE_STD, CXA_GPIO_POLARITY_NONINVERTED);
	GPIO_setQualificationMode(rxPinIn, GPIO_QUAL_ASYNC);

//    //
//    // Initialize interrupt controller and vector table.
//    // SAD:  These functions are called in NomadMainRev3_DL.c at lines 64 & 65
//    Interrupt_initModule();
//    Interrupt_initVectorTable();

    SCI_performSoftwareReset(SCIA_BASE); // SCI_XBASE could be an argument, but SCIA_BASE (only other option) is used for Bluetooth
    DEVICE_DELAY_US(10000);
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, baudRate_bpsIn, (SCI_CONFIG_WLEN_8 |
                                                                  SCI_CONFIG_STOP_ONE |
                                                                  SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_enableFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
    SCI_performSoftwareReset(SCIA_BASE);

	// setup our ioStream (last once everything is setup)
	cxa_ioStream_init(&usartIn->super.ioStream);
	cxa_ioStream_bind(&usartIn->super.ioStream, ioStream_cb_readByte_SCIA, ioStream_cb_writeBytes_SCIA, (void*)usartIn);
}


void cxa_tiC2K_usart_init_HH_BT(cxa_tiC2K_usart_t *const usartIn, const uint32_t baudRate_bpsIn,
                               const uint32_t txPinConfigIn, const uint32_t txPinIn,
                               const uint32_t rxPinConfigIn, const uint32_t rxPinIn,
                               const uint32_t ctsPinConfigIn, const uint32_t ctsPinIn)
//void cxa_esp32_usart_init_HH(cxa_esp32_usart_t *const usartIn, uart_port_t uartIdIn, const uint32_t baudRate_bpsIn,
//                            const gpio_num_t txPinIn, const gpio_num_t rxPinIn,
//                            const gpio_num_t rtsPinIn, const gpio_num_t ctsPinIn)
{
    cxa_assert(usartIn);

    // Setup TX_BTRx Pin
    cxa_tiC2K_gpio_t usartTxGpio;
    cxa_tiC2K_gpio_init_output(&usartTxGpio, txPinConfigIn, txPinIn, GPIO_CORE_CPU1, GPIO_PIN_TYPE_STD, CXA_GPIO_POLARITY_NONINVERTED, 1);
    GPIO_setQualificationMode(txPinIn, GPIO_QUAL_ASYNC);

    // Setup RX_BTTx Pin
    cxa_tiC2K_gpio_t usartRxGpio;
    cxa_tiC2K_gpio_init_input(&usartRxGpio, rxPinConfigIn, rxPinIn, GPIO_CORE_CPU1, GPIO_PIN_TYPE_STD, CXA_GPIO_POLARITY_NONINVERTED); //Customized for BT implementation
    GPIO_setQualificationMode(rxPinIn, GPIO_QUAL_ASYNC);

    // Setup CTS_BTRTS Pin
    cxa_tiC2K_gpio_t usartCTSGpio;
    GPIO_setAnalogMode(ctsPinIn, GPIO_ANALOG_DISABLED); //Customized for BT implementation
    cxa_tiC2K_gpio_init_input(&usartCTSGpio, rxPinConfigIn, rxPinIn, GPIO_CORE_CPU1, GPIO_PIN_TYPE_STD, CXA_GPIO_POLARITY_NONINVERTED);
    GPIO_setQualificationMode(ctsPinIn, GPIO_QUAL_6SAMPLE); //Customised for BT implementation

    // Setup RTS_BTCTS Pin //Customized for BT implementation
    // NOTE:  This pin is held low on the radio, so there is no need to configure an RTS input pin herein hardware
//    cxa_tiC2K_gpio_t usartRxGpio;
//    cxa_tiC2K_gpio_init_input(&usartRxGpio, rxPinConfigIn, rxPinIn, GPIO_CORE_CPU1, GPIO_PIN_TYPE_STD, CXA_GPIO_POLARITY_NONINVERTED);
//    GPIO_setQualificationMode(rxPinIn, GPIO_QUAL_ASYNC);

    //RTS_BTCTS
    // None, held low

//    //
//    // Initialize interrupt controller and vector table.
//    // SAD:  These functions are called in NomadMainRev3_DL.c at lines 64 & 65
//    Interrupt_initModule();
//    Interrupt_initVectorTable();

    SCI_performSoftwareReset(SCIB_BASE); // SCI_XBASE could be an argument, but SCIA_BASE (only other option) is used for Bluetooth
//    DEVICE_DELAY_US(10000);
    SCI_setConfig(SCIB_BASE, DEVICE_LSPCLK_FREQ, baudRate_bpsIn, (SCI_CONFIG_WLEN_8 |
                                                                  SCI_CONFIG_STOP_ONE |
                                                                  SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIB_BASE);
    SCI_resetRxFIFO(SCIB_BASE);
    SCI_resetTxFIFO(SCIB_BASE);
    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_enableFIFO(SCIB_BASE);
    SCI_enableModule(SCIB_BASE);
    SCI_performSoftwareReset(SCIB_BASE);

    // Reset the BGM121
    GPIO_writePin(11U, 0); // 0-Reset, 1-Active

    //See BLEIO.c for previous interrupt-driven setup of radio.

    // setup our ioStream (last once everything is setup)
    cxa_ioStream_init(&usartIn->super.ioStream);
    cxa_ioStream_bind(&usartIn->super.ioStream, ioStream_cb_readByte_SCIB, ioStream_cb_writeBytes_SCIB, (void*)usartIn);

    cxa_runLoop_addTimedEntry(CXA_RUNLOOP_THREADID_DEFAULT, 1000, NULL, runLoopCb_fifoStatusPrint, NULL);
}

// ******** local function implementations ********
static cxa_ioStream_readStatus_t ioStream_cb_readByte_SCIA(uint8_t *const byteOut, void *const userVarIn)
{
	cxa_tiC2K_usart_t* usartIn = (cxa_tiC2K_usart_t*)userVarIn;
	cxa_assert(usartIn);

	// TODO: try to read a byte from the serial port
	// return CXA_IOSTREAM_READSTAT_ERROR if you had an error
	// return CXA_IOSTREAM_READSTAT_NODATA if there is no byte available
	// return CXA_IOSTREAM_READSTAT_GOTDATA if a byte was read (don't forget to store it in *byteOut)

	// Assume Error  (TODO:  Strengthen Error Error Handling?)
	cxa_ioStream_readStatus_t retVal = CXA_IOSTREAM_READSTAT_ERROR;
	if(SCI_getRxFIFOStatus(SCIA_BASE) == SCI_FIFO_RX0)
    {
        retVal = CXA_IOSTREAM_READSTAT_NODATA;
    }
	else
	{
	    *byteOut = ((uint8_t) SCI_readCharNonBlocking(SCIA_BASE));
	    retVal = CXA_IOSTREAM_READSTAT_GOTDATA;
	}

	return retVal;
}


static bool ioStream_cb_writeBytes_SCIA(void* buffIn, size_t bufferSize_bytesIn, void *const userVarIn)
{
	cxa_tiC2K_usart_t* usartIn = (cxa_tiC2K_usart_t*)userVarIn;
	cxa_assert(usartIn);

	// TODO: try to write the bytes in the buffIn buffer to the serial port
	// return true if successful
	// return false on failure

    SCI_writeCharArray(SCIA_BASE, (uint16_t*) buffIn,
                       (uint16_t) bufferSize_bytesIn);

	return true;
}

static cxa_ioStream_readStatus_t ioStream_cb_readByte_SCIB(uint8_t *const byteOut, void *const userVarIn)
{
    cxa_tiC2K_usart_t* usartIn = (cxa_tiC2K_usart_t*)userVarIn;
    cxa_assert(usartIn);

    // TODO: try to read a byte from the serial port
    // return CXA_IOSTREAM_READSTAT_ERROR if you had an error
    // return CXA_IOSTREAM_READSTAT_NODATA if there is no byte available
    // return CXA_IOSTREAM_READSTAT_GOTDATA if a byte was read (don't forget to store it in *byteOut)

    // Assume Error  (TODO:  Strengthen Error Error Handling?)
    cxa_ioStream_readStatus_t retVal = CXA_IOSTREAM_READSTAT_ERROR;
    if(SCI_getRxFIFOStatus(SCIB_BASE) == SCI_FIFO_RX0)
    {
        retVal = CXA_IOSTREAM_READSTAT_NODATA;
    }
    else
    {
        *byteOut = ((uint8_t) SCI_readCharNonBlocking(SCIB_BASE));
        cxa_logger_stepDebug_msg("r 0x%02X", *byteOut);
        retVal = CXA_IOSTREAM_READSTAT_GOTDATA;
    }

    return retVal;
}


static bool ioStream_cb_writeBytes_SCIB(void* buffIn, size_t bufferSize_bytesIn, void *const userVarIn)
{
    cxa_tiC2K_usart_t* usartIn = (cxa_tiC2K_usart_t*)userVarIn;
    cxa_assert(usartIn);
    // TODO: try to write the bytes in the buffIn buffer to the serial port
    // return true if successful
    // return false on failure

    for( size_t i = 0; i < bufferSize_bytesIn; i++ )
    {
        //
        // Wait until space is available in the transmit FIFO.
        //
        while(SCI_getTxFIFOStatus(SCIB_BASE) >= SCI_FIFO_TX14)
        {
        }

        //
        // Send a char.
        //
        HWREGH(SCIB_BASE + SCI_O_TXBUF) = ((uint8_t*)buffIn)[i];
    }
    return true;
}

////
//// sciaTxISR - Disable the TXFF interrupt and print message asking
////             for two characters.
////
//__interrupt void
//scibTxISR(void)
//{
////    //
////    // Disable the TXRDY interrupt.
////    //
////    SCI_disableInterrupt(SCIA_BASE, SCI_INT_TXFF);
////
////    msg = "\r\nEnter two characters: \0";
////    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 26);
//
//    //
//    // Acknowledge the PIE interrupt.
//    //
//    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXFF); //[SAD] Added, but needed?
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
//}
//
////
//// sciaRxISR - Read two characters from the RXBUF and echo them back.
////
//__interrupt void
//scibRxISR(void)
//{
////    uint16_t receivedChar1, receivedChar2;
////
////    //
////    // Enable the TXFF interrupt again.
////    //
////    SCI_enableInterrupt(SCIA_BASE, SCI_INT_TXFF);
////
////    //
////    // Read two characters from the FIFO.
////    //
////    receivedChar1 = SCI_readCharBlockingFIFO(SCIA_BASE);
////    receivedChar2 = SCI_readCharBlockingFIFO(SCIA_BASE);
////
////    //
////    // Echo back the two characters.
////    //
////    msg = "  You sent: \0";
////    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 13);
////    SCI_writeCharBlockingFIFO(SCIA_BASE, receivedChar1);
////    SCI_writeCharBlockingFIFO(SCIA_BASE, receivedChar2);
//
//    //
//    // Clear the SCI RXFF interrupt and acknowledge the PIE interrupt.
//    //
//    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
//
////    counter++;
//}


static void runLoopCb_fifoStatusPrint(void* userVarIn)
{
    uint16_t rxStatus = SCI_getRxStatus(SCIB_BASE);
    cxa_logger_stepDebug_msg("%d  %04X", SCI_getRxFIFOStatus(SCIB_BASE), rxStatus);
    if( rxStatus & SCI_RXSTATUS_ERROR )
    {
        cxa_logger_stepDebug_msg("resetting");
        SCI_performSoftwareReset(SCIB_BASE);
    }
}
