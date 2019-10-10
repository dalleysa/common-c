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
static cxa_ioStream_readStatus_t ioStream_cb_readByte(uint8_t *const byteOut, void *const userVarIn);
static bool ioStream_cb_writeBytes(void* buffIn, size_t bufferSize_bytesIn, void *const userVarIn);

static void gpioCb_fallingEdge_cts(cxa_gpio_t *const gpioIn, bool newValIn, void* userVarIn);

static void handleRxInterrupt(cxa_tiC2K_usart_t *const usartIn);
static void handleTxCompleteInterrupt(cxa_tiC2K_usart_t *const usartIn);

#warning skyler
// refactor these to match your ISRs if needed (also change your implementations below)
__interrupt void sciaTxISR(void);
__interrupt void sciaRxISR(void);
__interrupt void scibTxISR(void);
__interrupt void scibRxISR(void);


// ********  local variable declarations *********
static cxa_tiC2K_usart_t* usartA = NULL;
static cxa_tiC2K_usart_t* usartB = NULL;


// ******** global function implementations ********
void cxa_tiC2K_usart_init_noHH(cxa_tiC2K_usart_t *const usartIn, const uint32_t baudRate_bpsIn,
                               uint32_t sciBaseIn,
							   const uint32_t txPinConfigIn, const uint32_t txPinIn,
							   const uint32_t rxPinConfigIn, const uint32_t rxPinIn)
{
	cxa_assert(usartIn);

	// save our references and setup our internal state
	usartIn->sciBase = sciBaseIn;
	usartIn->gpio_cts = NULL;
	cxa_fixedFifo_initStd(&usartIn->txFifo, CXA_FF_ON_FULL_DROP, usartIn->txFifo_raw);
	cxa_fixedFifo_initStd(&usartIn->rxFifo, CXA_FF_ON_FULL_DROP, usartIn->rxFifo_raw);
	usartIn->rxOverflow = false;

	// record this for interrupts
	if( usartIn->sciBase == SCIA_BASE ) usartA = usartIn;
	else if( usartIn->sciBase == SCIB_BASE ) usartB = usartIn;

	// Setup TX Pin
	cxa_tiC2K_gpio_t usartTxGpio;
	cxa_tiC2K_gpio_init_output(&usartTxGpio, txPinConfigIn, txPinIn, GPIO_CORE_CPU1, GPIO_PIN_TYPE_STD, CXA_GPIO_POLARITY_NONINVERTED, 1);
	GPIO_setQualificationMode(txPinIn, GPIO_QUAL_ASYNC);

	// Setup RX Pin
	cxa_tiC2K_gpio_t usartRxGpio;
	cxa_tiC2K_gpio_init_input(&usartRxGpio, rxPinConfigIn, rxPinIn, GPIO_CORE_CPU1, GPIO_PIN_TYPE_STD, CXA_GPIO_POLARITY_NONINVERTED);
	GPIO_setQualificationMode(rxPinIn, GPIO_QUAL_ASYNC);

	// configure the hardware
	SCI_performSoftwareReset(usartIn->sciBase);
	DEVICE_DELAY_US(10000);
	SCI_setConfig(usartIn->sciBase, DEVICE_LSPCLK_FREQ, baudRate_bpsIn, (SCI_CONFIG_WLEN_8 |
			SCI_CONFIG_STOP_ONE |
			SCI_CONFIG_PAR_NONE));
	SCI_resetChannels(usartIn->sciBase);
	SCI_resetRxFIFO(usartIn->sciBase);
	SCI_resetTxFIFO(usartIn->sciBase);
	SCI_clearInterruptStatus(usartIn->sciBase, SCI_INT_TXRDY | SCI_INT_RXRDY_BRKDT);//Skyler removed SCI_INT_TXFF | SCI_INT_RXFF);
	SCI_enableFIFO(usartIn->sciBase);
	SCI_enableModule(usartIn->sciBase);
	SCI_performSoftwareReset(usartIn->sciBase);

	// setup our ioStream (last once everything is setup)
	cxa_ioStream_init(&usartIn->super.ioStream);
	cxa_ioStream_bind(&usartIn->super.ioStream, ioStream_cb_readByte, ioStream_cb_writeBytes, (void*)usartIn);

#warning skyler
	// enable your receive interrupt here
	SCI_enableInterrupt(usartIn->sciBase, SCI_INT_RXRDY_BRKDT);//Skyler-Added
}


void cxa_tiC2K_usart_init_HH_CTSonly(cxa_tiC2K_usart_t *const usartIn, const uint32_t baudRate_bpsIn,
							 	 	 uint32_t sciBaseIn,
									 const uint32_t txPinConfigIn, const uint32_t txPinIn,
									 const uint32_t rxPinConfigIn, const uint32_t rxPinIn,
									 cxa_gpio_t *const gpio_ctsIn)
{
	cxa_assert(usartIn);
	cxa_assert(gpio_ctsIn);

	// save our references and setup our internal state
	usartIn->sciBase = sciBaseIn;
	usartIn->gpio_cts = gpio_ctsIn;
	cxa_fixedFifo_initStd(&usartIn->txFifo, CXA_FF_ON_FULL_DROP, usartIn->txFifo_raw);
	cxa_fixedFifo_initStd(&usartIn->rxFifo, CXA_FF_ON_FULL_DROP, usartIn->rxFifo_raw);
	usartIn->rxOverflow = false;

	// record this for interrupts
	if( usartIn->sciBase == SCIA_BASE ) usartA = usartIn;
	else if( usartIn->sciBase == SCIB_BASE ) usartB = usartIn;

	// Setup TX_BTRx Pin
	cxa_tiC2K_gpio_t usartTxGpio;
	cxa_tiC2K_gpio_init_output(&usartTxGpio, txPinConfigIn, txPinIn, GPIO_CORE_CPU1, GPIO_PIN_TYPE_STD, CXA_GPIO_POLARITY_NONINVERTED, 1);
	GPIO_setQualificationMode(txPinIn, GPIO_QUAL_ASYNC);

	// Setup RX_BTTx Pin
	cxa_tiC2K_gpio_t usartRxGpio;
	cxa_tiC2K_gpio_init_input(&usartRxGpio, rxPinConfigIn, rxPinIn, GPIO_CORE_CPU1, GPIO_PIN_TYPE_STD, CXA_GPIO_POLARITY_NONINVERTED);
	GPIO_setQualificationMode(rxPinIn, GPIO_QUAL_ASYNC);

	/*Skyler-Commented Out
	// Setup CTS_BTRTS Pin
	cxa_tiC2K_gpio_t usartCTSGpio;
	GPIO_setAnalogMode(ctsPinIn, GPIO_ANALOG_DISABLED);
	cxa_tiC2K_gpio_init_input(&usartCTSGpio, rxPinConfigIn, rxPinIn, GPIO_CORE_CPU1, GPIO_PIN_TYPE_STD, CXA_GPIO_POLARITY_NONINVERTED);
	GPIO_setQualificationMode(ctsPinIn, GPIO_QUAL_6SAMPLE);
	*/

	//skyler-disable global interrupts? (see sci_ex2_interrupts.c)
//	DINT;

	//skyler-initialize interrupt controller and vector table?  (Already performed in NomadMainRev3_DL.c) (see sci_ex2_interrupts.c)

//    Interrupt_initModule();
//    Interrupt_initVectorTable(); //load default vector table.
//    IER = 0x0000; // [SAD]  Added in an attempt to follow SCI interrupt example
//    IFR = 0x0000;

	//skyler-Map the ISR to the wake interrupt? (see sci_ex2_interrupts.c)

//    if( usartIn->sciBase == SCIA_BASE )
//    {
//        Interrupt_register(INT_SCIA_TX, sciaTxISR);
//        Interrupt_register(INT_SCIA_RX, sciaRxISR);
//    }
//    else if( usartIn->sciBase == SCIB_BASE )
//    {
//        Interrupt_register(INT_SCIB_TX, sciaTxISR);
//        Interrupt_register(INT_SCIB_RX, sciaRxISR);
//    }

	// Initialize SCIA and its FIFO
	SCI_performSoftwareReset(usartIn->sciBase);
	// Configure the SCI
	SCI_setConfig(usartIn->sciBase, DEVICE_LSPCLK_FREQ, baudRate_bpsIn, (SCI_CONFIG_WLEN_8 |
			SCI_CONFIG_STOP_ONE |
			SCI_CONFIG_PAR_NONE));
	SCI_resetChannels(usartIn->sciBase);
	SCI_resetRxFIFO(usartIn->sciBase); //skyler-needed?
	SCI_resetTxFIFO(usartIn->sciBase); //skyler-needed?
	SCI_clearInterruptStatus(usartIn->sciBase, SCI_INT_TXRDY | SCI_INT_RXRDY_BRKDT);
	SCI_enableFIFO(usartIn->sciBase); //skyler-needed?
	SCI_enableModule(usartIn->sciBase);
	SCI_performSoftwareReset(usartIn->sciBase);

    //skyler-Set the transmit FIFO level to 0 and the receive FIFO level to 2?
//	SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0, SCI_FIFO_RX2);

    //skyler-Enable the TXRDY and RXRDY_BRKDT interrupts?  (Or only enable RXRDY below?)
//    SCI_enableInterrupt(usartIn->sciBase, SCI_INT_TXRDY | SCI_INT_RXRDY_BRKDT);

    // skyler-Clear the SCI interrupts before enabling them.
//    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF | SCI_INT_RXFF);

    // skyler-Enable the interrupts in the PIE: Group 9 interrupts 1 & 2.
//    Interrupt_enable(INT_SCIA_RX);
//    Interrupt_enable(INT_SCIA_TX);
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

    // skyler-Enable global interrupts?
//    EINT;

	// setup our ioStream (last once everything is setup) //skyler-setup doesn't seem complete here?
	cxa_ioStream_init(&usartIn->super.ioStream);
	cxa_ioStream_bind(&usartIn->super.ioStream, ioStream_cb_readByte, ioStream_cb_writeBytes, (void*)usartIn);

	// enable our GPIO interrupt
	cxa_gpio_enableInterrupt(gpio_ctsIn, CXA_GPIO_INTERRUPTTYPE_FALLING_EDGE, gpioCb_fallingEdge_cts, (void*)usartIn);
#warning skyler
	// enable your receive interrupt here
	SCI_enableInterrupt(usartIn->sciBase, SCI_INT_RXRDY_BRKDT);//Skyler-Added, but this could be done above?
}


// ******** local function implementations ********
static cxa_ioStream_readStatus_t ioStream_cb_readByte(uint8_t *const byteOut, void *const userVarIn)
{
	cxa_tiC2K_usart_t* usartIn = (cxa_tiC2K_usart_t*)userVarIn;
	cxa_assert(usartIn);

	// see if we've had an overflow
	if( usartIn->rxOverflow )
	{
		usartIn->rxOverflow = false;
		return CXA_IOSTREAM_READSTAT_ERROR;
	}

	// if we made it here, no errors, read a byte from our sofware rxFifo
	cxa_ioStream_readStatus_t retVal = CXA_IOSTREAM_READSTAT_NODATA;
	uint8_t rxByte;
	if( cxa_fixedFifo_dequeue(&usartIn->rxFifo, rxByte) )
	{
		if( byteOut != NULL ) *byteOut = rxByte;
		retVal = CXA_IOSTREAM_READSTAT_GOTDATA;
	}

	return retVal;
}


static bool ioStream_cb_writeBytes(void* buffIn, size_t bufferSize_bytesIn, void *const userVarIn)
{
	cxa_tiC2K_usart_t* usartIn = (cxa_tiC2K_usart_t*)userVarIn;
	cxa_assert(usartIn);

	// queue the bytes for transmit
	if( !cxa_fixedFifo_bulkQueue(&usartIn->txFifo, buffIn, bufferSize_bytesIn) ) return false;

#warning skyler
	// check your TXRDY interrupt here...if it's already enabled, return true
	if( (SCI_getInterruptStatus(usartIn->sciBase) & SCI_INT_TXRDY) == SCI_INT_TXRDY) return true;//skyler-added

	// if we made it here, we're not currently transmitting
#warning skyler
	// generally speaking, enabling your TXRDY interrupt here _should_ cause an
	// immediate vector to that ISR where we can handle things
	SCI_enableInterrupt(usartIn->sciBase, SCI_INT_TXRDY);//skyler-added

	return true;
}


static void gpioCb_fallingEdge_cts(cxa_gpio_t *const gpioIn, bool newValIn, void* userVarIn)
{
	cxa_tiC2K_usart_t *const usartIn = (cxa_tiC2K_usart_t *const)userVarIn;
	cxa_assert(usartIn);

	// if we have bytes to send AND the receiver says we're good to go, send them
	if( !cxa_fixedFifo_isEmpty(&usartIn->txFifo) &&
		((usartIn->gpio_cts == NULL) || (newValIn == 0)) )
	{
#warning skyler
		// generally speaking, enabling your TXRDY interrupt here _should_ cause an
		// immediate vector to that ISR where we can handle things
	    SCI_enableInterrupt(usartIn->sciBase, SCI_INT_TXRDY);//skyler-added
	}
}


static void handleRxInterrupt(cxa_tiC2K_usart_t *const usartIn)
{
	if( usartIn == NULL ) return;

	uint8_t rxByte = ((uint8_t)SCI_readCharNonBlocking(usartIn->sciBase));
	if( !cxa_fixedFifo_queue(&usartIn->rxFifo, rxByte) )
	{
		usartIn->rxOverflow = true;
	}
}


static void handleTxCompleteInterrupt(cxa_tiC2K_usart_t *const usartIn)
{
	if( usartIn == NULL ) return;

	// we can't use the hardware FIFO because it doesn't have flow control
	// if we have a byte in the fifo, wait for it to be sent
	if( SCI_getTxFIFOStatus(usartIn->sciBase) >= SCI_FIFO_TX1 ) return;

	// we're ready to transmit another byte...see if we have one
	if( cxa_fixedFifo_isEmpty(&usartIn->txFifo) )
	{
#warning skyler
		// disable your TXRDY interrupt here
	    SCI_disableInterrupt(usartIn->sciBase, SCI_INT_TXRDY);//skyler-added
		return;
	}

	// we have a byte to transmit...make sure our receiver is ready (if we can)
	if( (usartIn->gpio_cts == NULL) || (cxa_gpio_getValue(usartIn->gpio_cts) == 0) )
	{
		// receiver is good (or no flow control) send our next byte immediately
		uint8_t txByte;
		cxa_fixedFifo_dequeue(&usartIn->txFifo, txByte);
		HWREGH(usartIn->sciBase + SCI_O_TXBUF) = txByte;
	}
	else
	{
		// receiver is requesting a pause...

#warning skyler
		// disable your TXRDY interrupt here
	    SCI_disableInterrupt(usartIn->sciBase, SCI_INT_TXRDY);//skyler-added
	}
}


// ******** interrupt handlers ********
__interrupt void sciaTxISR(void)
{
	handleTxCompleteInterrupt(usartA);

#warning skyler
	// not sure what all you need to do in terms of clearing flags to mark this interupt as serviced

    // Clear the SCI RXFF interrupt and acknowledge the PIE interrupt.
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXRDY);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}


__interrupt void sciaRxISR(void)
{
	handleRxInterrupt(usartA);

#warning skyler
	// not sure what all you need to do in terms of clearing flags to mark this interupt as serviced

    // Clear the SCI RXFF interrupt and acknowledge the PIE interrupt.
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXRDY_BRKDT);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}


__interrupt void scibTxISR(void)
{
	handleTxCompleteInterrupt(usartB);

#warning skyler
	// not sure what all you need to do in terms of clearing flags to mark this interupt as serviced

    // Clear the SCI RXFF interrupt and acknowledge the PIE interrupt.
	SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXRDY);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}


__interrupt void scibRxISR(void)
{
	handleRxInterrupt(usartB);

#warning skyler
	// not sure what all you need to do in terms of clearing flags to mark this interupt as serviced

    // Clear the SCI RXFF interrupt and acknowledge the PIE interrupt.
    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_RXRDY_BRKDT);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
