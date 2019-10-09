/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 * @author Christopher Armenio
 */
#ifndef CXA_DUMMY_USART_H_
#define CXA_DUMMY_USART_H_


// ******** includes ********
#include <stdbool.h>
#include <stdio.h>
#include <cxa_config.h>
#include <cxa_usart.h>
#include <cxa_fixedFifo.h>
#include <cxa_gpio.h>
#include <gpio.h>
#include <cxa_tiC2K_gpio.h>


// ******** global macro definitions ********
#ifndef CXA_TIC2K_USART_TX_FIFO_SIZE_BYTES
#define CXA_TIC2K_USART_TX_FIFO_SIZE_BYTES 32
#endif

#ifndef CXA_TIC2K_USART_RX_FIFO_SIZE_BYTES
#define CXA_TIC2K_USART_RX_FIFO_SIZE_BYTES 16
#endif



// ******** global type definitions *********
/**
 * @public
 * @brief "Forward" declaration of the cxa_tiC2K_usart_t object
 */
typedef struct cxa_tiC2K_usart cxa_tiC2K_usart_t;


/**
 * @private
 */
struct cxa_tiC2K_usart
{
	cxa_usart_t super;

	cxa_fixedFifo_t txFifo;
	uint8_t txFifo_raw[CXA_TIC2K_USART_TX_FIFO_SIZE_BYTES];

	cxa_fixedFifo_t rxFifo;
	uint8_t rxFifo_raw[CXA_TIC2K_USART_RX_FIFO_SIZE_BYTES];

	bool rxOverflow;

	uint32_t sciBase;
	cxa_gpio_t *gpio_cts;
};


// ******** global function prototypes ********
/**
 * @public
 * @brief Initializes the specified USART for no hardware handshaking using the specified baud rate.
 *
 * @param[in] usartIn pointer to a pre-allocated USART object
 * @param[in] baudRate_bpsIn the desired baud rate, in bits-per-second
 */
void cxa_tiC2K_usart_init_noHH(cxa_tiC2K_usart_t *const usartIn, const uint32_t baudRate_bpsIn,
                               uint32_t sciBaseIn,
							   const uint32_t txPinConfigIn, const uint32_t txPinIn,
							   const uint32_t rxPinConfigIn, const uint32_t rxPinIn);


void cxa_tiC2K_usart_init_HH_CTSonly(cxa_tiC2K_usart_t *const usartIn, const uint32_t baudRate_bpsIn,
                                     uint32_t sciBaseIn,
									 const uint32_t txPinConfigIn, const uint32_t txPinIn,
									 const uint32_t rxPinConfigIn, const uint32_t rxPinIn,
									 cxa_gpio_t *const gpio_ctsIn);

#endif
