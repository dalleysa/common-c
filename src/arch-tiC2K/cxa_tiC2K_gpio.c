/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 * @author Christopher Armenio
 */
#include "cxa_tiC2K_gpio.h"


// ******** includes ********
#include <cxa_assert.h>


#define CXA_LOG_LEVEL				CXA_LOG_LEVEL_TRACE
#include <cxa_logger_implementation.h>


// ******** local macro definitions ********
#define MAX_NUM_INTERRUPTS		2 //skyler-there are 5 available external interrupts


// ******** local type definitions ********


// ******** local function prototypes ********
static void initSystem(void);

static void scm_setDirection(cxa_gpio_t *const superIn, const cxa_gpio_direction_t dirIn);
static cxa_gpio_direction_t scm_getDirection(cxa_gpio_t *const superIn);
static void scm_setPolarity(cxa_gpio_t *const superIn, const cxa_gpio_polarity_t polarityIn);
static cxa_gpio_polarity_t scm_getPolarity(cxa_gpio_t *const superIn);
static void scm_setValue(cxa_gpio_t *const superIn, const bool valIn);
static bool scm_getValue(cxa_gpio_t *const superIn);
static bool scm_enableInterrupt(cxa_gpio_t *const superIn, cxa_gpio_interruptType_t intTypeIn, cxa_gpio_cb_onInterrupt_t cbIn, void* userVarIn);

#warning skyler
// refactor this prototype and the implementation as neeeded
void handleExtGpioInterrupt(void);


// ********  local variable declarations *********
static bool isSystemInit = false;
static cxa_array_t gpiosWithInterrupts;
static cxa_gpio_t* gpiosWithInterrupts_raw[MAX_NUM_INTERRUPTS];


// ******** global function implementations ********
void cxa_tiC2K_gpio_init_input(cxa_tiC2K_gpio_t *const gpioIn, const uint32_t pinConfigIn, const uint32_t pinIn, const GPIO_CoreSelect coreIn, const uint32_t pinTypeIn, const cxa_gpio_polarity_t polarityIn)
{
	cxa_assert(gpioIn);

	// make sure our system is initialized
	if( !isSystemInit ) initSystem();

	// save our references / internal state
	gpioIn->pinConfig = pinConfigIn;
	gpioIn->pin = pinIn;
	gpioIn->core = coreIn;
	gpioIn->pinType = pinTypeIn;
	gpioIn->polarity = polarityIn;
	cxa_logger_init(&gpioIn->logger, "gpio");

	// initialize our super class
	cxa_gpio_init(&gpioIn->super, scm_setDirection, scm_getDirection, scm_setPolarity, scm_getPolarity, scm_setValue, scm_getValue, scm_enableInterrupt);

	GPIO_setMasterCore(pinIn, coreIn);
    GPIO_setPinConfig(pinConfigIn);
    // set our initial direction
    cxa_gpio_setDirection(&gpioIn->super, CXA_GPIO_DIR_INPUT);
    GPIO_setPadConfig(pinIn, pinTypeIn);
}


void cxa_tiC2K_gpio_init_output(cxa_tiC2K_gpio_t *const gpioIn, const uint32_t pinConfigIn, const uint32_t pinIn, const GPIO_CoreSelect coreIn, const uint32_t pinTypeIn, const cxa_gpio_polarity_t polarityIn, const bool initValIn)
{
	cxa_assert(gpioIn);

	// make sure our system is initialized
	if( !isSystemInit ) initSystem();

	// save our references / internal state
    gpioIn->pinConfig = pinConfigIn;
    gpioIn->pin = pinIn;
    gpioIn->core = coreIn;
    gpioIn->pinType = pinTypeIn;
	gpioIn->polarity = polarityIn;
//    gpioIn->outVal = (uint32_t) initValIn;
	cxa_logger_init(&gpioIn->logger, "gpio");

	// initialize our super class
	cxa_gpio_init(&gpioIn->super, scm_setDirection, scm_getDirection, scm_setPolarity, scm_getPolarity, scm_setValue, scm_getValue, NULL);

    GPIO_setPinConfig(pinConfigIn);
    GPIO_setMasterCore(pinIn, coreIn);
    GPIO_setPadConfig(pinIn, pinTypeIn);

	// set our initial value (before we set direction to avoid glitches)
	cxa_gpio_setValue(&gpioIn->super, initValIn);

	// now set our direction
	cxa_gpio_setDirection(&gpioIn->super, CXA_GPIO_DIR_OUTPUT);

    /*Example LED initialization.
     * Will need to implement setPinConfig, setMasterCore, and setPadConfig

       //BLUELED1 B
       GPIO_setPinConfig(GPIO_56_GPIO56);
       GPIO_setMasterCore(56U, GPIO_CORE_CPU1);
       GPIO_setPadConfig(56U, GPIO_PIN_TYPE_STD);
       GPIO_setDirectionMode(56U, GPIO_DIR_MODE_OUT);
       GPIO_writePin(56U, 1);
     */
}


// ******** local function implementations ********
static void initSystem(void)
{
	cxa_array_initStd(&gpiosWithInterrupts, gpiosWithInterrupts_raw);

	isSystemInit = true;
}


static void scm_setDirection(cxa_gpio_t *const superIn, const cxa_gpio_direction_t dirIn)
{
	cxa_assert(superIn);
	cxa_assert( (dirIn == CXA_GPIO_DIR_INPUT) ||
				(dirIn == CXA_GPIO_DIR_OUTPUT) );

	// get a pointer to our class
	cxa_tiC2K_gpio_t *const gpioIn = (cxa_tiC2K_gpio_t *const)superIn;

	cxa_logger_trace(&gpioIn->logger, "new direction: %s", (dirIn == CXA_GPIO_DIR_INPUT) ? "input" : "output");

	// TODO: set your direction here
	// From TI's gpio.h
	/* GPIO_setDirectionMode(uint32_t pin, GPIO_Direction pinIO); */
	// From esp32 example
	/* gpio_set_direction(gpioIn->pinNum, (dirIn == CXA_GPIO_DIR_OUTPUT) ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT); */

	GPIO_setDirectionMode(gpioIn->pin, (dirIn == CXA_GPIO_DIR_OUTPUT) ? GPIO_DIR_MODE_OUT : GPIO_DIR_MODE_IN);

    gpioIn->dir = dirIn;
}


static cxa_gpio_direction_t scm_getDirection(cxa_gpio_t *const superIn)
{
	cxa_assert(superIn);

	// get a pointer to our class
	cxa_tiC2K_gpio_t *const gpioIn = (cxa_tiC2K_gpio_t *const)superIn;

	return gpioIn->dir;
}


static void scm_setPolarity(cxa_gpio_t *const superIn, const cxa_gpio_polarity_t polarityIn)
{
	cxa_assert(superIn);
	cxa_assert( (polarityIn == CXA_GPIO_POLARITY_NONINVERTED) ||
				(polarityIn == CXA_GPIO_POLARITY_INVERTED) );

	// get a pointer to our class
	cxa_tiC2K_gpio_t *const gpioIn = (cxa_tiC2K_gpio_t *const)superIn;

	gpioIn->polarity = polarityIn;
}


static cxa_gpio_polarity_t scm_getPolarity(cxa_gpio_t *const superIn)
{
	cxa_assert(superIn);

	// get a pointer to our class
	cxa_tiC2K_gpio_t *const gpioIn = (cxa_tiC2K_gpio_t *const)superIn;

	return gpioIn->polarity;
}


static void scm_setValue(cxa_gpio_t *const superIn, const bool valIn)
{
	cxa_assert(superIn);

	// get a pointer to our class
	cxa_tiC2K_gpio_t *const gpioIn = (cxa_tiC2K_gpio_t *const)superIn;

	cxa_logger_trace(&gpioIn->logger, "new logic value: %d", valIn);

	// TODO: set your value here
    // From TI's gpio.h
    /* GPIO_writePin(uint32_t pin, uint32_t outVal); */
    // From esp32 example
    /* gpio_set_level(gpioIn->pinNum, (gpioIn->polarity == CXA_GPIO_POLARITY_INVERTED) ? !valIn : valIn); */

//	GPIO_writePin(gpioIn->pin, (gpioIn->polarity == CXA_GPIO_POLARITY_INVERTED) ? !gpioIn->outVal : gpioIn->outVal);
    GPIO_writePin(gpioIn->pin, (gpioIn->polarity == CXA_GPIO_POLARITY_INVERTED) ? ((uint32_t)!valIn) : ((uint32_t)valIn));
	gpioIn->lastVal = (gpioIn->polarity == CXA_GPIO_POLARITY_INVERTED) ? ((uint32_t)!valIn) : ((uint32_t)valIn);
}


static bool scm_getValue(cxa_gpio_t *const superIn)
{
	cxa_assert(superIn);

	// get a pointer to our class
	cxa_tiC2K_gpio_t *const gpioIn = (cxa_tiC2K_gpio_t *const)superIn;

	// TODO: return your GPIO logic value (don't forget to adjust for polarity)
    // From TI's gpio.h
    /* GPIO_readPin(uint32_t pin); */
    // From esp32 example
    /* bool retVal = (gpioIn->dir == CXA_GPIO_DIR_INPUT) ? gpio_get_level(gpioIn->pinNum) : gpioIn->lastVal;
       return (gpioIn->polarity == CXA_GPIO_POLARITY_INVERTED) ? !retVal : retVal; */

	bool retVal = (gpioIn->dir == CXA_GPIO_DIR_INPUT) ? GPIO_readPin(gpioIn->pin) : gpioIn->lastVal;
	return (gpioIn->polarity == CXA_GPIO_POLARITY_INVERTED) ? !retVal : retVal;
}


static bool scm_enableInterrupt(cxa_gpio_t *const superIn, cxa_gpio_interruptType_t intTypeIn, cxa_gpio_cb_onInterrupt_t cbIn, void* userVarIn)
{
    size_t gpiosWithInterruptsCount;
    GPIO_ExternalIntNum extIntNum;

	cxa_assert(superIn);

	// get a pointer to our class
	cxa_tiC2K_gpio_t *const gpioIn = (cxa_tiC2K_gpio_t *const)superIn;

	// save a reference to the gpio
	cxa_assert(cxa_array_append(&gpiosWithInterrupts, (void*)&gpioIn));

	// determine how many gpios have interrupts
	gpiosWithInterruptsCount = (gpiosWithInterrupts.insertIndex-1);

	// assign external interrupt
    #warning are these interrupts claimed elsewhere?)
	if      (gpiosWithInterruptsCount == 1) extIntNum = GPIO_INT_XINT1;
	else if (gpiosWithInterruptsCount == 2) extIntNum = GPIO_INT_XINT2;
	else if (gpiosWithInterruptsCount == 3) extIntNum = GPIO_INT_XINT3;
	else if (gpiosWithInterruptsCount == 4) extIntNum = GPIO_INT_XINT4;
	else if (gpiosWithInterruptsCount == 5) extIntNum = GPIO_INT_XINT5;
	else; //error

	// TODO: enable interrupts for your GPIO here
	#warning skyler
	GPIO_setInterruptPin(gpioIn->pin,extIntNum);

	//GPIO_setInterruptType(extIntNum, intType);
    if      (intTypeIn == CXA_GPIO_INTERRUPTTYPE_RISING_EDGE) GPIO_setInterruptType(extIntNum,GPIO_INT_TYPE_FALLING_EDGE);
    else if (intTypeIn == CXA_GPIO_INTERRUPTTYPE_FALLING_EDGE) GPIO_setInterruptType(extIntNum,GPIO_INT_TYPE_RISING_EDGE);
    else if (intTypeIn == CXA_GPIO_INTERRUPTTYPE_ONCHANGE) GPIO_setInterruptType(extIntNum,GPIO_INT_TYPE_BOTH_EDGES);
    else; //error

	GPIO_enableInterrupt(extIntNum);

	return true;
}


// ******** interrupt handlers ********
#warning skyler
void handleExtGpioInterrupt(void)
{
	// iterate through your gpios that have enabled interrupts to find the right one
	cxa_array_iterate(&gpiosWithInterrupts, currGpio, cxa_gpio_t*)
	{
		if( currGpio == NULL ) continue;

		// TODO: test some GPIO flag register to determine _which_ gpio triggered the interrupt
		#warning skyler
		if( false )
		{
			bool currVal = scm_getValue(*currGpio);
			if( (*currGpio)->cbs.onInterrupt != NULL ) (*currGpio)->cbs.onInterrupt(*currGpio,
															currVal,
															(*currGpio)->cbs.userVar);
		}
	}
}
