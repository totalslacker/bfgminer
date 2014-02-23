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

#define BMHASHER_QUEUE_MEMORY 0x20

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
	uint16_t		seq;
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

	amountSent = bmhasher_write(fd, packet, sendLength);
	return (sendLength == amountSent) ? 0 : -1;
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

struct bmhasher_detection_state {
	uint8_t moduleCount;
};

static
bool bmhasher_detect_one(const char * const devpath)
{
	uint16_t clock = 200;
	BMPacket hello = {0};
	struct bmhasher_detection_state * detectState;
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

	const int expectlen = 1;
	if (responsePacket.header.length < expectlen)
	{
		applog(LOG_DEBUG, "%s: USB_INIT response too short on %s (%d < %d)",
		       __func__, devpath, (int) responsePacket.header.length, expectlen);
		goto err;
	}

	
	if (serial_claim_v(devpath, &bmhasher_drv))
		return false;
	
	applog(LOG_DEBUG, "%s: Opening BMHasher on %s",
	        __func__, devpath);

	detectState = malloc(sizeof(*detectState));
	*detectState = (struct bmhasher_detection_state)
	{
		.moduleCount = 1,		// FIXME: actual number of modules!
	};

	struct cgpu_info * const cgpu = malloc(sizeof(*cgpu));
	*cgpu = (struct cgpu_info){
		.drv = &bmhasher_drv,
		.device_path = strdup(devpath),
		.deven = DEV_ENABLED,
		// .procs = (pmsg->chipaddr * pmsg->coreaddr),
		.procs = 1,	// FIXME: One processor per module!
		.threads = 1,
		.device_data = detectState,
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

typedef unsigned long bmhasher_isn_t;

#define	MAX_MODULES		32

struct bmhasher_module_state {
	struct cgpu_info *proc;
	uint8_t addr;
	uint8_t last_seq;
	bmhasher_isn_t last_isn;
	bmhasher_isn_t last2_isn;
	bool has_pending;
	unsigned queued;
	// float voltages[HASHFAST_MAX_VOLTAGES];
};

struct bmhasher_chain_state {
	uint8_t module_count;
	int fd;
	struct bmhasher_module_state modules[MAX_MODULES];
};

static
bool bmhasher_init(struct thr_info * const master_thr)
{
	struct cgpu_info * const dev = master_thr->cgpu, *proc;
	struct bmhasher_chain_state * const chainstate = calloc(1, sizeof(*chainstate));
	struct bmhasher_module_state * modstate;
	struct bmhasher_detection_state * detectState = dev->device_data;
	int i;

	applog(LOG_DEBUG, "%s: moduleCount %d", __func__, detectState->moduleCount);

	*chainstate = (struct bmhasher_chain_state)
	{
		.module_count = 1,
		.fd = serial_open(dev->device_path, 115200, 1, true),
	};
	
	applog(LOG_DEBUG, "%s: chainstate->fd=%d", __func__, chainstate->fd);

	for (i = 0; i < detectState->moduleCount; ++i)
	{
		modstate = &chainstate->modules[i];
		*modstate = (struct bmhasher_module_state)
		{
			.addr = i,		// FIXME: We don't know this...
		};
	}
	
	for ((i = 0), (proc = dev); proc; ++i, (proc = proc->next_proc))
	{
		struct thr_info * const thr = proc->thr[0];
		// const bool core_is_working = pmsg->data[0x20 + (i / 8)] & (1 << (i % 8));
		
		// if (!core_is_working)
		// 	proc->deven = DEV_RECOVER_DRV;
		proc->device_data = chainstate;
		modstate = &chainstate->modules[i];
		thr->cgpu_data = modstate;
		modstate->proc = proc;
	}
	
	// TODO: actual clock = [12,13]
	
	timer_set_now(&master_thr->tv_poll);
	applog(LOG_DEBUG, "%s: finishing", __func__);
	return true;
}

static
bool bmhasher_queue_append(struct thr_info * const thr, struct work * const work)
{
	struct cgpu_info * const proc = thr->cgpu;
	struct bmhasher_chain_state * const chainstate = proc->device_data;
	const int fd = chainstate->fd;
	struct bmhasher_module_state * const modstate = thr->cgpu_data;
	bmhasher_isn_t isn;
	uint8_t seq;
	WorkPacket workPacket = {0};
	
	applog(LOG_DEBUG, "%s: has_pending=%d", __func__, modstate->has_pending);

	if (modstate->has_pending)
	{
		thr->queue_full = true;
		return false;
	}
	
	isn = ++modstate->last_isn;
	seq = ++modstate->last_seq;
	work->device_id = seq;
	modstate->last_isn = isn;

	workPacket.header.type = kWorkPacketType;
	workPacket.header.length = 46;
	workPacket.seq = seq;
	memcpy(workPacket.midstate, work->midstate, 32);
	memcpy(workPacket.data, &work->data[64], 12);
	modstate->has_pending = true;

	if (SendPacket(fd, modstate->addr, (BMPacket *) &workPacket) < 0)
	{
		applog(LOG_DEBUG, "%s: error sending work packet", __func__);
		return false;
	}

	DL_APPEND(thr->work, work);
	if (modstate->queued > BMHASHER_QUEUE_MEMORY)
	{
		struct work * const old_work = thr->work;
		DL_DELETE(thr->work, old_work);
		free_work(old_work);
	}
	else
	{
		++modstate->queued;
	}

	return true;
}

static
void bmhasher_poll(struct thr_info * const master_thr)
{
	struct cgpu_info * const dev = master_thr->cgpu;
	struct timeval tv_timeout;
	timer_set_delay_from_now(&tv_timeout, 10000);
#if 0
	while (true)
	{
		if (!hashfast_poll_msg(master_thr))
		{
			applog(LOG_DEBUG, "%s poll: No more messages", dev->dev_repr);
			break;
		}
		if (timer_passed(&tv_timeout, NULL))
		{
			applog(LOG_DEBUG, "%s poll: 10ms timeout met", dev->dev_repr);
			break;
		}
	}
#else
	applog(LOG_DEBUG, "%s", __func__);
#endif
	
	timer_set_delay_from_now(&master_thr->tv_poll, 100000);
}

static
void bmhasher_queue_flush(struct thr_info * const thr)
{
#if 0
	struct cgpu_info * const proc = thr->cgpu;
	struct hashfast_dev_state * const devstate = proc->device_data;
	const int fd = devstate->fd;
	struct hashfast_core_state * const cs = thr->cgpu_data;
	uint8_t cmd[HASHFAST_HEADER_SIZE];
	uint16_t hdata = 2;
	if ((!thr->work) || stale_work(thr->work->prev, true))
	{
		applog(LOG_DEBUG, "%"PRIpreprv": Flushing both active and pending work",
		       proc->proc_repr);
		hdata |= 1;
	}
	else
		applog(LOG_DEBUG, "%"PRIpreprv": Flushing pending work",
		       proc->proc_repr);
	hashfast_send_msg(fd, cmd, HFOP_ABORT, cs->chipaddr, cs->coreaddr, hdata, 0);
#else
	applog(LOG_DEBUG, "%s", __func__);
#endif
}

struct device_drv bmhasher_drv = {
	.dname = "bmhasher",
	.name = "BMH",
	
	.lowl_match = bmhasher_lowl_match,
	.lowl_probe = bmhasher_lowl_probe,
	
	.thread_init = bmhasher_init,
	
	.minerloop = minerloop_queue,
	.queue_append = bmhasher_queue_append,
	.queue_flush = bmhasher_queue_flush,
	.poll = bmhasher_poll,
	
	// .get_api_stats = hashfast_api_stats,
	
#ifdef HAVE_CURSES
	// .proc_wlogprint_status = hashfast_wlogprint_status,
#endif
};
