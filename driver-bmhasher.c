/*
 * Copyright 2014 Shannon Holland
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <utlist.h>

#include "deviceapi.h"
#include "logging.h"
#include "lowlevel.h"
#include "lowl-vcom.h"
#include "util.h"

BFG_REGISTER_DRIVER(bmhasher_drv)



struct device_drv bmhasher_drv = {
	.dname = "bmhasher",
	.name = "BMH",
	
	// .lowl_match = hashfast_lowl_match,
	// .lowl_probe = hashfast_lowl_probe,
	
	// .thread_init = hashfast_init,
	
	// .minerloop = minerloop_queue,
	// .queue_append = hashfast_queue_append,
	// .queue_flush = hashfast_queue_flush,
	// .poll = hashfast_poll,
	
	// .get_api_stats = hashfast_api_stats,
	
#ifdef HAVE_CURSES
	// .proc_wlogprint_status = hashfast_wlogprint_status,
#endif
};
