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

#define LogFail(file, line) { applog(LOG_WARNING, "Failure in %s at line %d", file, line); }
#define ExitWithErrorIf(cond, label) { if (cond) goto label; }
#define ExitWithResultIf(cond, resultCode) { if (cond) { result = resultCode; if (resultCode < 0) { LogFail(__FILE__, __LINE__); } goto CLEANUP; } }

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
	kRequestStatusPacketType,
	kStopPacketType,
};

#define BMH_MARKER	0x55

typedef struct BMPacketHeader
{
	uint8_t		marker;
	uint8_t		type;
	uint8_t		address;
	uint8_t		flags;
	uint16_t	length;
	uint16_t	dataCheck;
	uint16_t	headerCheck;
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

typedef struct WorkResult
{
	uint32_t		nonce;
	uint16_t		complete		: 1;
	uint16_t		hasNonce		: 1;
	uint16_t		reserved		: 14;
	uint16_t		seq;
} WorkResult;

#define	STATUS_MAX_RESULTS		16

typedef struct StatusResponsePacket
{
	BMPacketHeader	header;
	uint8_t			remainingWork;
	uint8_t			desiredWork;
	uint8_t			remainingResults;
	uint8_t			resultsCount;
	WorkResult		workResults[STATUS_MAX_RESULTS];
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

uint16_t CRCPayload(BMPacket * packet)
{
	uint16_t crc = 0xFFFF;

	// crc payload buffer
	if (packet->header.length > 0)
	{
		crc = crc16(crc, packet->payload, packet->header.length);
	}

	return crc;
}

uint16_t CRCPacketHeader(BMPacketHeader * header)
{
	uint16_t crc = 0xFFFF;

	// hash header except crc
	crc = crc16(crc, header, sizeof(BMPacketHeader) - 2);

	return crc;
}

int BuildPacket(BMPacket * packet, int type, int address, int length)
{
	int result;

	packet->header.marker = BMH_MARKER;
	packet->header.type = type;
	packet->header.address = address;
	packet->header.length = length;

	// crc the payload first
	packet->header.dataCheck = CRCPayload(packet);

	// now crc the header
	packet->header.headerCheck = CRCPacketHeader(&packet->header);

	result = 0;
CLEANUP:
	if (result < 0)
	{
		printf("BuildPacket: result=%d\n", result);
	}

	return result;
}

int CheckPacketHeader(BMPacketHeader * header)
{
	int result;
	uint16_t crc;

	crc = CRCPacketHeader(header);
	ExitWithResultIf(crc != header->headerCheck, -1);

	result = 0;
CLEANUP:
	if (result < 0)
	{
		printf("CheckPacketHeader: result=%d crc=0x%04x headerCheck=0x%04x\n", result, crc, header->headerCheck);
	}

	return result;
}

int CheckPacketPayload(BMPacket * packet)
{
	int result;
	uint16_t crc;

	crc = CRCPayload(packet);
	ExitWithResultIf(crc != packet->header.dataCheck, -1);

	result = 0;
CLEANUP:
	if (result < 0)
	{
		printf("CheckPacketPayload: result=%d crc=0x%04x header.dataCheck=0x%04x\n", result, crc, packet->header.dataCheck);
	}

	return result;
}

int CheckPacket(BMPacket * packet)
{
	int result;

	result = CheckPacketHeader(&packet->header);
	ExitWithResultIf(result < 0, result);

	result = CheckPacketPayload(packet);
	ExitWithResultIf(result < 0, result);

	result = 0;
CLEANUP:
	if (result < 0)
	{
		printf("CheckPacket: result=%d\n", result);
	}

	return result;
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

int SendPacket(int fd, int type, int moduleIndex, int length, BMPacket * packet)
{
	int sendLength;
	int amountSent;

	// FIXME: Set this correctly...
	packet->header.address = moduleIndex;
	BuildPacket(packet, type, moduleIndex, length);
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

#define	MAX_MODULES		8

struct bmhasher_detection_state {
	uint8_t		moduleCount;
	uint16_t	moduleMap[MAX_MODULES];
};

static
bool bmhasher_detect_one(const char * const devpath)
{
	uint16_t clock = 200;
	BMPacket hello = {0};
	struct bmhasher_detection_state * detectState;
	HelloResponsePacket responsePacket;

	const int fd = serial_open(devpath, 115200, 10, true);
	if (fd == -1)
	{
		applog(LOG_DEBUG, "%s: Failed to open %s", __func__, devpath);
		return false;
	}

	applog(LOG_DEBUG, "%s: devpath=%s", __func__, devpath);

	drv_set_defaults(&bmhasher_drv, bmhasher_set_device_funcs_probe, &clock, devpath, detectone_meta_info.serial, 1);


	detectState = malloc(sizeof(*detectState));
	memset(detectState, 0, sizeof(*detectState));

	// see how many modules we can connect to
	for (int moduleAddress = 0; moduleAddress < MAX_MODULES; moduleAddress++)
	{
		applog(LOG_DEBUG, "bmhasher_detect_one: detecting moduleAddress=%d", moduleAddress);
		SendPacket(fd, kHelloPacketType, moduleAddress, 0, &hello);

		while (true)
		{
			if (ReceivePacket(fd, (BMPacket *) &responsePacket))
			{
				if (responsePacket.header.type == kHelloPacketType)
				{
					applog(LOG_DEBUG, "bmhasher_detect_one: got response from moduleAddress=%d", moduleAddress);
					detectState->moduleMap[detectState->moduleCount++] = moduleAddress;
					break;
				}
				else
				{
					applog(LOG_DEBUG, "bmhasher_detect_one: invalid packet (type=%d) from moduleAddress=%d",
						responsePacket.header.type, moduleAddress);
				}
			}
			else
			{
				applog(LOG_DEBUG, "bmhasher_detect_one: no response from moduleAddress=%d", moduleAddress);
				break;
			}
		}
	}
	
	applog(LOG_DEBUG, "bmhasher_detect_one: %d modules detected", detectState->moduleCount);

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

	struct cgpu_info * const cgpu = malloc(sizeof(*cgpu));
	*cgpu = (struct cgpu_info){
		.drv = &bmhasher_drv,
		.device_path = strdup(devpath),
		.deven = DEV_ENABLED,
		// .procs = (pmsg->chipaddr * pmsg->coreaddr),
		.procs = detectState->moduleCount,
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

struct bmhasher_module_state {
	struct cgpu_info *proc;
	uint8_t addr;
	uint16_t last_seq;
	bmhasher_isn_t last_isn;
	bmhasher_isn_t last2_isn;
	int queue_depth;
	int desired_queue_depth;
	unsigned queued;
	// float voltages[HASHFAST_MAX_VOLTAGES];
};

struct bmhasher_chain_state {
	uint8_t module_count;
	uint8_t last_module;
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
		.module_count = detectState->moduleCount,
		.fd = serial_open(dev->device_path, 115200, 1, true),
	};
	
	applog(LOG_DEBUG, "%s: chainstate->fd=%d", __func__, chainstate->fd);

	for (i = 0; i < detectState->moduleCount; ++i)
	{
		modstate = &chainstate->modules[i];
		*modstate = (struct bmhasher_module_state)
		{
			.queue_depth = 0,
			.desired_queue_depth = 1,			// FIXME: Get this from the module
			.addr = detectState->moduleMap[i],
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

uint32_t get_diff(double diff)
{
	uint32_t n_bits;
	int shift = 29;
	double f = (double) 0x0000ffff / diff;
	while (f < (double) 0x00008000) {
		shift--;
		f *= 256.0;
	}
	while (f >= (double) 0x00800000) {
		shift++;
		f /= 256.0;
	}
	n_bits = (int) f + (shift << 24);
	return n_bits;
}

static
bool bmhasher_queue_append(struct thr_info * const thr, struct work * const work)
{
	struct cgpu_info * const proc = thr->cgpu;
	struct bmhasher_chain_state * const chainstate = proc->device_data;
	const int fd = chainstate->fd;
	struct bmhasher_module_state * const modstate = thr->cgpu_data;
	bmhasher_isn_t isn;
	uint16_t seq;
	WorkPacket workPacket = {0};
	
	applog(LOG_DEBUG, "%s: addr=%d queue_depth=%d desired_queue_depth=%d",
		__func__, modstate->addr, modstate->queue_depth, modstate->desired_queue_depth);

	if (modstate->queue_depth >= modstate->desired_queue_depth)
	{
		thr->queue_full = true;
		return false;
	}
	
	isn = ++modstate->last_isn;
	seq = ++modstate->last_seq;
	work->device_id = seq;
	modstate->last_isn = isn;

	workPacket.seq = seq;
	memcpy(workPacket.midstate, work->midstate, 32);
	memcpy(workPacket.data, &work->data[64], 12);
	modstate->queue_depth++;

#if 0
	char midstateStr[65];
	char dataStr[65];

	bin2hex(midstateStr, work->midstate, 32);
	bin2hex(dataStr, &work->data[64], 12);
	applog(LOG_DEBUG, "seq=0x%04x midstate=%s data=%s", workPacket.seq, midstateStr, dataStr);

	bin2hex(midstateStr, workPacket.midstate, 32);
	bin2hex(dataStr, workPacket.data, 12);
	applog(LOG_DEBUG, "seq=0x%04x packet midstate=%s data=%s", workPacket.seq, midstateStr, dataStr);
#endif

	// uint32_t diff = get_diff(work->sdiff);
	// applog(LOG_DEBUG, "address=%d difficulty=0x%08x", modstate->addr, diff);

	if (SendPacket(fd, kWorkPacketType, modstate->addr, 46, (BMPacket *) &workPacket) < 0)
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
struct cgpu_info *bmhasher_find_proc(struct thr_info * const master_thr, int modaddr)
{
	struct cgpu_info *proc = master_thr->cgpu;
	struct bmhasher_chain_state * const chainstate = proc->device_data;

	applog(LOG_DEBUG, "%s: modaddr=%d module_count=%d", __func__, modaddr, chainstate->module_count);

	struct bmhasher_module_state * modstate = NULL;
	for (int i = 0; i < chainstate->module_count; i++)
	{
		if (chainstate->modules[i].addr == modaddr)
		{
			modstate = &chainstate->modules[i];
			break;
		}
	}

	applog(LOG_DEBUG, "%s: modaddr=%d modstate=%p", __func__, modaddr, modstate);

	return modstate->proc;
}

static
void bmhasher_submit_nonce(struct thr_info * const thr, struct work * const work, const uint32_t nonce)
{
	struct cgpu_info * const proc = thr->cgpu;
	struct bmhasher_module_state * const modstate = thr->cgpu_data;
	
	applog(LOG_DEBUG, "%"PRIpreprv": Found nonce for seq %04x (last=%04x): %08lx",
	       proc->proc_repr, (unsigned) work->device_id, (unsigned) modstate->last_seq,
	       (unsigned long) nonce);

#if 0
	{
		char hash_str[65];
		int ok;

		ok = test_nonce2(work, nonce);
		if (ok == 0)
		{
			applog(LOG_DEBUG, "%"PRIpreprv": FOUND GOOD NONCE!!!", proc->proc_repr);
		}
		applog(LOG_DEBUG, "%"PRIpreprv": test_nonce=%d", proc->proc_repr, ok);
		bin2hex(hash_str, work->hash, 32);
		applog(LOG_DEBUG, "%"PRIpreprv": seq=%04x hash=%s", proc->proc_repr, (unsigned) work->device_id, hash_str);
	}
#endif

	submit_nonce(thr, work, nonce);
}

static
void bmhasher_poll(struct thr_info * const master_thr)
{
	struct cgpu_info * const proc = master_thr->cgpu;
	struct bmhasher_chain_state * const chainstate = proc->device_data;
	const int fd = chainstate->fd;
	struct timeval tv_timeout;
	BMPacket statusPacket;
	StatusResponsePacket statusReponsePacket;
	int moduleIndex;
	int moduleAddress;

	// FIXME: This needs to iterate through all modules! Or perhaps do a single module each time it's called?
	moduleIndex = chainstate->last_module++;
	if (chainstate->last_module > chainstate->module_count)
	{
		chainstate->last_module = 0;
	}
	moduleAddress = chainstate->modules[moduleIndex].addr;

	// FIXME: This timeout is likely way too long? 10ms?
	timer_set_delay_from_now(&tv_timeout, 10000);

	applog(LOG_DEBUG, "%s: moduleAddress=%d", __func__, moduleAddress);
	SendPacket(fd, kRequestStatusPacketType, moduleAddress, 0, &statusPacket);

	while (true)
	{
		if (!ReceivePacket(fd, (BMPacket *) &statusReponsePacket))
		{
			applog(LOG_DEBUG, "%s: Failed to parse response", __func__);
		}

		if (statusReponsePacket.header.type == kRequestStatusPacketType)
		{
#if 0
			applog(LOG_DEBUG, "%s: got status response packet", __func__);
			applog(LOG_DEBUG, "%s: remainingWork=%u", __func__, statusReponsePacket.remainingWork);
			applog(LOG_DEBUG, "%s: desiredWork=%u", __func__, statusReponsePacket.desiredWork);
			applog(LOG_DEBUG, "%s: remainingResults=%u", __func__, statusReponsePacket.remainingResults);
			applog(LOG_DEBUG, "%s: resultsCount=%u", __func__, statusReponsePacket.resultsCount);
#endif
			applog(LOG_DEBUG, "%s: remainingWork=%u", __func__, statusReponsePacket.remainingWork);
			applog(LOG_DEBUG, "%s: desiredWork=%u", __func__, statusReponsePacket.desiredWork);

			// submit the work results
			struct cgpu_info * const proc = bmhasher_find_proc(master_thr, statusReponsePacket.header.address);
			if (unlikely(!proc))
			{
				applog(LOG_ERR, "%s: Unknown module address %u",
				       __func__, (unsigned) statusReponsePacket.header.address);
				inc_hw_errors_only(master_thr);
				continue;
			}
			struct thr_info * const thr = proc->thr[0];
			struct bmhasher_module_state * const modstate = thr->cgpu_data;
			unsigned nonces_found = 0;

			// unblock the queue if the module is needing work
			if (statusReponsePacket.desiredWork > 0)
			{
				modstate->queue_depth = 0;
				modstate->desired_queue_depth = statusReponsePacket.desiredWork;
				thr->queue_full = false;
			}

			// go post any results
			for (int index = 0; index < statusReponsePacket.resultsCount; index++)
			{
				struct work *work;
				uint16_t seq;
				WorkResult * workResult = &statusReponsePacket.workResults[index];

				seq = workResult->seq;

				// find the matching work
				DL_SEARCH_SCALAR(thr->work, work, device_id, seq);
				if (unlikely(!work))
				{
					applog(LOG_WARNING, "%"PRIpreprv": Unknown seq %04x (last=%04x)",
					       proc->proc_repr, (unsigned) seq, (unsigned) modstate->last_seq);
					inc_hw_errors2(thr, NULL, &workResult->nonce);
					continue;
				}
				
				// was a nonce found?
				if (workResult->hasNonce)
				{
					uint32_t nonce;

					applog(LOG_DEBUG, "%"PRIpreprv": Found nonce seq %04x", proc->proc_repr, (unsigned) work->device_id);
					nonce = htobe32(workResult->nonce);
					// nonce = workResult->nonce;
					nonces_found++;
					bmhasher_submit_nonce(thr, work, nonce);
				}

				// is the work done?
				if (workResult->complete)
				{
					// job is complete, but we leave it in queue where it's deleted at append
					// FIXME: Verify that this is the best thing to do...
					applog(LOG_DEBUG, "%"PRIpreprv": Work Complete seq %04x", proc->proc_repr, (unsigned) seq);
				}
			}

			break;
		}

		if (timer_passed(&tv_timeout, NULL))
		{
			applog(LOG_DEBUG, "%s poll: 10ms timeout met", proc->dev_repr);
			break;
		}
	}
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
