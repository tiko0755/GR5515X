
#ifndef __GR55xx_HAL_CALENDAR_MODULE_H__
#define __GR55xx_HAL_CALENDAR_MODULE_H__

#include "gr55xx_hal_def.h"
#include "gr55xx_hal_calendar.h"

hal_status_t hal_calendar_module_enable_seconds_irq(void);

hal_status_t hal_calendar_module_disable_seconds_irq(void);

hal_status_t hal_calendar_module_set_time(calendar_time_t time);

hal_status_t hal_calendar_module_get_time(calendar_time_t* p_time);

hal_status_t hal_calendar_module_init(void);

void hal_calendar_module_seconds_callback(void);

#endif
