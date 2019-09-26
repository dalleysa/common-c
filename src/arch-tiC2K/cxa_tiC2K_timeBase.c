/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 * @author Christopher Armenio
 */
#include <cxa_timeBase.h>


// ******** includes ********
#include <Externals.h>
#include <cxa_assert.h>


// ******** local macro definitions ********


// ******** local type definitions ********


// ******** local function prototypes ********


// ********  local variable declarations *********


// ******** global function implementations ********
void cxa_tiC2K_timeBase_init(void)
{
}


uint32_t cxa_timeBase_getCount_us(void)
{
    return cpuTimer1IntCount*1000; // cpuTimer1IntCount is in ms, multiply by 1000 to get microsecond units.
}


uint32_t cxa_timeBase_getMaxCount_us(void)
{
	return UINT32_MAX;
}


// ******** local function implementations ********
