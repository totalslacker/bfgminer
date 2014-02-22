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


/********** temporary helper for hexdumping SPI traffic */
#define DEBUG_HEXDUMP 1
static void dbghexdump(char *prefix, uint8_t *buff, int len)
{
#if DEBUG_HEXDUMP
	static char line[2048];
	char *pos = line;
	int i;
	if (len < 1)
		return;

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) {
		if (i > 0 && (i % 32) == 0)
			pos += sprintf(pos, "\n\t");
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(LOG_DEBUG, "%s", line);
#endif
}


enum PacketTypes
{
	kHelloPacketType,
	kWorkPacketType,
	kRequestNoncesPacketType,
	kStopPacketType,
};

#define BMH_MARKER	0x55

typedef struct BMPacketHeader
{
	uint8_t		marker;
	uint8_t		type;
	uint8_t		address;
	uint8_t		length;
	uint16_t	flags;
	uint16_t	crc16;
} BMPacketHeader;

typedef struct BMPacket
{
	BMPacketHeader	header;
	uint8_t			payload[];
} BMPacket;

typedef struct HelloResponsePacket
{
	BMPacketHeader	header;
	uint8_t			desiredQueueLength;
} HelloResponsePacket;

typedef struct WorkPacket
{
	BMPacketHeader	header;
	uint8_t			midstate[32];
	uint8_t			data[12];
} WorkPacket;

typedef struct StatusResponsePacket
{
	BMPacketHeader	header;
	uint8_t			remainingWork;
	uint8_t			desiredWork;
	uint8_t			remainingNonces;
	uint8_t			nonceCount;
	uint32_t		nonces[16];
} StatusResponsePacket;

uint16_t crc16(uint16_t crcval, void *data_p, int count)
{
    /* CRC-16 Routine for processing multiple part data blocks.
     * Pass 0 into 'crcval' for first call for any given block; for
     * subsequent calls pass the CRC returned by the previous call. */
    int 		xx;
    uint8_t * 	ptr = data_p;

    while (count-- > 0)
    {
        crcval = (uint16_t)( crcval ^ (uint16_t)(((uint16_t) *ptr++) << 8));
        for (xx=0; xx < 8; xx++)
        {
            if (crcval & 0x8000)
            {
            	crcval=(uint16_t) ((uint16_t) (crcval << 1) ^ 0x1021);
            }
            else 
            {
            	crcval = (uint16_t) (crcval << 1);
            }
        }
    }

    return (crcval);
}

int CRCPacket(BMPacket * packet, uint16_t * crc)
{
	*crc = 0xFFFF;

	// hash header except crc
	*crc = crc16(*crc, packet, sizeof(BMPacketHeader) - 2);

	// hash payload
	if (packet->header.length > 0)
	{
		*crc = crc16(*crc, packet->payload, packet->header.length - 2);
	}

	return 0;
}

int BuildPacket(BMPacket * packet, int length)
{
	packet->header.marker = BMH_MARKER;
	packet->header.length = length;
	CRCPacket(packet, &packet->header.crc16);

	return 0;
}

int CheckPacket(BMPacket * packet)
{
	uint16_t crc;

	if (packet->header.marker != BMH_MARKER)
	{
		return -1;
	}

	CRCPacket(packet, &crc);

	return (crc == packet->header.crc16) ? 0 : -1;
}

static
ssize_t bmhasher_write(const int fd, void * const buf, size_t bufsz)
{
	const ssize_t rv = write(fd, buf, bufsz);
	if (true || (opt_debug && opt_dev_protocol) || unlikely(rv != bufsz))
	{
		const int e = errno;
		char hex[(bufsz * 2) + 1];
		bin2hex(hex, buf, bufsz);
		if (rv < 0)
			applog(LOG_WARNING, "%s fd=%d: SEND (%s) => %d errno=%d(%s)",
			       "bmhasher", fd, hex, (int)rv, e, bfg_strerror(e, BST_ERRNO));
		else
		if (rv < bufsz)
			applog(LOG_WARNING, "%s fd=%d: SEND %.*s(%s)",
			       "bmhasher", fd, (int)(rv * 2), hex, &hex[rv * 2]);
		else
		if (rv > bufsz)
			applog(LOG_WARNING, "%s fd=%d: SEND %s => +%d",
			       "bmhasher", fd, hex, (int)(rv - bufsz));
		else
			applog(LOG_DEBUG, "%s fd=%d: SEND %s",
			       "bmhasher", fd, hex);
	}
	return rv;
}

static
ssize_t bmhasher_read(const int fd, void * const buf, size_t bufsz)
{
	const ssize_t rv = serial_read(fd, buf, bufsz);
	if (opt_debug && opt_dev_protocol && rv)
	{
		char hex[(rv * 2) + 1];
		bin2hex(hex, buf, rv);
		applog(LOG_DEBUG, "%s fd=%d: RECV %s",
		       "bmhasher", fd, hex);
	}
	return rv;
}
int SendPacket(int fd, int moduleIndex, BMPacket * packet)
{
	int sendLength;
	int amountSent;

	// FIXME: Set this correctly...
	packet->header.address = moduleIndex;
	BuildPacket(packet, packet->header.length);
	sendLength = packet->header.length + sizeof(BMPacketHeader);

	dbghexdump("SendPacket", (uint8_t *) packet, sendLength);

	return (sendLength == bmhasher_write(fd, packet, sendLength));
}

int ReceivePacket(int fd, BMPacket * packet)
{
	int len, err, tried;
	int rxLength;

	applog(LOG_DEBUG, "ReceivePacket");

	// just keep scanning for a marker byte
	// FIXME: This can clearly be better - at least attempt to read the whole header in 
	while (true)
	{
		if (bmhasher_read(fd, &packet->header.marker, 1) != 1)
		{
			applog(LOG_DEBUG, "ReceivePacket: error reading marker");
			return false;
		}

		if (packet->header.marker == BMH_MARKER)
		{
			break;
		}
		applog(LOG_DEBUG, "ReceivePacket: waiting marker - received %02x", packet->header.marker);
	}

	rxLength = sizeof(BMPacketHeader) - 1;
	if (bmhasher_read(fd, &packet->header.type, rxLength) != rxLength)
	{
		applog(LOG_DEBUG, "ReceivePacket: error reading remaining header");
		return false;
	}

	// read the payload if any
	if (packet->header.length > 0)
	{
		if (bmhasher_read(fd, &packet->payload, packet->header.length) != packet->header.length)
		{
			applog(LOG_DEBUG, "ReceivePacket: error reading payload");
			return false;
		}
	}

	return sizeof(BMPacketHeader) + packet->header.length;
}

static const struct bfg_set_device_definition bmhasher_set_device_funcs_probe[] = {
	// {"clock", hashfast_set_clock, "clock frequency (can only be set at startup, with --set-device)"},
	{NULL},
};

static
bool bmhasher_detect_one(const char * const devpath)
{
	uint16_t clock = 200;
	BMPacket hello = {0};
	HelloResponsePacket responsePacket;

	const int fd = serial_open(devpath, 115200, 100, true);
	if (fd == -1)
	{
		applog(LOG_DEBUG, "%s: Failed to open %s", __func__, devpath);
		return false;
	}

	applog(LOG_DEBUG, "%s: devpath=%s", __func__, devpath);

	drv_set_defaults(&bmhasher_drv, bmhasher_set_device_funcs_probe, &clock, devpath, detectone_meta_info.serial, 1);

	applog(LOG_DEBUG, "bmhasher_detect_one: sending hello packet");
	hello.header.type = kHelloPacketType;
	hello.header.length = 0;
	SendPacket(fd, 0, &hello);

	do {
		if (!ReceivePacket(fd, (BMPacket *) &responsePacket))
		{
			applog(LOG_DEBUG, "%s: Failed to parse response on %s",
			        __func__, devpath);
			serial_close(fd);
			goto err;
		}
	} while (responsePacket.header.type != kHelloPacketType);

	serial_close(fd);

	const int expectlen = sizeof(HelloResponsePacket) - sizeof(BMPacketHeader);
	if (responsePacket.header.length < expectlen)
	{
		applog(LOG_DEBUG, "%s: USB_INIT response too short on %s (%d < %d)",
		       __func__, devpath, (int) responsePacket.header.length, expectlen);
		goto err;
	}

	
	if (serial_claim_v(devpath, &bmhasher_drv))
		return false;
	
	struct cgpu_info * const cgpu = malloc(sizeof(*cgpu));
	*cgpu = (struct cgpu_info){
		.drv = &bmhasher_drv,
		.device_path = strdup(devpath),
		.deven = DEV_ENABLED,
		// .procs = (pmsg->chipaddr * pmsg->coreaddr),
		.procs = (4),
		.threads = 1,
		.device_data = NULL,
		.cutofftemp = 100,
	};
	return add_cgpu(cgpu);

err:
	return false;
}

static
bool bmhasher_lowl_probe(const struct lowlevel_device_info * const info)
{
	return vcom_lowl_probe_wrapper(info, bmhasher_detect_one);
}

static
bool bmhasher_lowl_match(const struct lowlevel_device_info * const info)
{
	if (!lowlevel_match_id(info, &lowl_vcom, 0, 0))
		return false;
	return (info->manufacturer && strstr(info->manufacturer, "HashFast"));
}


struct device_drv bmhasher_drv = {
	.dname = "bmhasher",
	.name = "BMH",
	
	.lowl_match = bmhasher_lowl_match,
	.lowl_probe = bmhasher_lowl_probe,
	
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
