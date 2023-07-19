/**********************************************************
filename: pin.c
**********************************************************/

/************************����ͷ�ļ�***************************************************/
#include "pin.h"

/**********************************************************
 Private function
**********************************************************/
static app_io_pin_state_t pinX_read(PIN_rsrc_t*);
static uint16_t pinX_write(PIN_rsrc_t*, app_io_pin_state_t level);
static uint16_t pinX_toggle(PIN_rsrc_t*);

/**********************************************************
 Public function
**********************************************************/
void PIN_DEV_Setup(
	pinX_t *d,
	app_io_type_t           type,           /**< Specifies IO type */
	uint32_t                pin,           	/**< Specifies the IO pins to be configured. */
	app_io_mode_t           mode,           /**< Specifies the IO mode for the selected pins. */
	app_io_pull_t           pull,           /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
	app_io_mux_t       		mux
){
	PIN_rsrc_t *r = &d->rsrc;
	app_io_init_t iParam;
	
	r->type = type;
	r->pin = pin;
	r->mode = mode;
	r->pull = pull;
	r->mux = mux;
	
	iParam.mode = mode;
	iParam.pin = pin;
	iParam.pull = pull;
	iParam.mux = mux;

	app_io_init(type, &iParam);

	// register functions
	d->Read = pinX_read;
	d->Write = pinX_write;
	d->Toggle = pinX_toggle;
}

static app_io_pin_state_t pinX_read(PIN_rsrc_t* r){
	return( app_io_read_pin(r->type, r->pin));
}

static uint16_t pinX_write(PIN_rsrc_t* r, app_io_pin_state_t level){
	return (app_io_write_pin(r->type, r->pin, level));
}

static uint16_t pinX_toggle(PIN_rsrc_t* r){
	return (app_io_toggle_pin(r->type, r->pin));
}


/**********************************************************
 == THE END ==
**********************************************************/
