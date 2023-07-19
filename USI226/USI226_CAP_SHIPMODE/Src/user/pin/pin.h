/**********************************************************
filename: pin.h
**********************************************************/
#ifndef _PIN_H_
#define _PIN_H_

#include "stdint.h"
#include "app_io.h"

/*****************************************************************************
 @ typedefs
****************************************************************************/
#pragma pack(push,4)		// push current align bytes, and then set 4 bytes align

typedef struct{
	app_io_type_t           type;           /**< Specifies IO type */
	uint32_t                pin;           	/**< Specifies the IO pins to be configured. */
	app_io_mode_t           mode;           /**< Specifies the IO mode for the selected pins. */
	app_io_pull_t           pull;           /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
	app_io_mux_t       			mux;
}PIN_rsrc_t;

typedef struct{
	PIN_rsrc_t rsrc;
	//basic
	app_io_pin_state_t (*Read)(PIN_rsrc_t*);
	uint16_t (*Write)(PIN_rsrc_t*, app_io_pin_state_t level);
	uint16_t (*Toggle)(PIN_rsrc_t*);
}pinX_t;
#pragma pack(pop)		//recover align bytes from 4 bytes

void PIN_DEV_Setup(
	pinX_t *d,
	app_io_type_t           type,           /**< Specifies IO type */
	uint32_t                pin,           	/**< Specifies the IO pins to be configured. */
	app_io_mode_t           mode,           /**< Specifies the IO mode for the selected pins. */
	app_io_pull_t           pull,           /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
	app_io_mux_t       			mux
);

#endif
