/*
 * The Mass Storage protocol state machine in this file is based on mbed's
 * implementation. We augment it by adding Zephyr's USB transport and Storage
 * APIs.
 *
 * Copyright (c) 2010-2011 mbed.org, MIT License
 * Copyright (c) 2016 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


/**
 * @file
 * @brief Mass Storage device class driver
 *
 * Driver for USB Mass Storage device class driver
 */

#include <zephyr/init.h>
#include <errno.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/usb/class/usb_msc.h>
#include <zephyr/usb/usb_device.h>
#include <usb_descriptor.h>

#define LOG_LEVEL CONFIG_USB_MASS_STORAGE_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_msc);

/* max USB packet size */
// #define MAX_PACKET	CONFIG_MASS_STORAGE_BULK_EP_MPS

#define BLOCK_SIZE	512
#define DISK_THREAD_PRIO	-5

#define THREAD_OP_READ_QUEUED		1
#define THREAD_OP_WRITE_QUEUED		3
#define THREAD_OP_WRITE_DONE		4

#define MASS_STORAGE_IN_EP_ADDR		0x82
#define MASS_STORAGE_OUT_EP_ADDR	0x01

struct usb_mass_config {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_in_ep;
	struct usb_ep_descriptor if0_out_ep;
} __packed;

USBD_CLASS_DESCR_DEFINE(primary, 0) struct usb_mass_config mass_cfg = {
	/* Interface descriptor */
	.if0 = {
		.bLength = sizeof(struct usb_if_descriptor),
		.bDescriptorType = USB_DESC_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_BCC_MASS_STORAGE,
		.bInterfaceSubClass = SCSI_TRANSPARENT_SUBCLASS,
		.bInterfaceProtocol = BULK_ONLY_TRANSPORT_PROTOCOL,
		.iInterface = 0,
	},
	/* First Endpoint IN */
	.if0_in_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = MASS_STORAGE_IN_EP_ADDR,
		.bmAttributes = USB_DC_EP_BULK,
		.wMaxPacketSize =
			sys_cpu_to_le16(CONFIG_MASS_STORAGE_BULK_EP_MPS),
		.bInterval = 0x00,
	},
	/* Second Endpoint OUT */
	.if0_out_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = MASS_STORAGE_OUT_EP_ADDR,
		.bmAttributes = USB_DC_EP_BULK,
		.wMaxPacketSize =
			sys_cpu_to_le16(CONFIG_MASS_STORAGE_BULK_EP_MPS),
		.bInterval = 0x00,
	},
};

static volatile int thread_op;
static K_KERNEL_STACK_DEFINE(mass_thread_stack, CONFIG_MASS_STORAGE_STACK_SIZE);
static struct k_thread mass_thread_data;
static struct k_sem disk_wait_sem;
static volatile uint32_t defered_wr_sz;

/*
 * Keep block buffer larger than BLOCK_SIZE for the case
 * the dCBWDataTransferLength is multiple of the BLOCK_SIZE and
 * the length of the transferred data is not aligned to the BLOCK_SIZE.
 *
 * Align for cases where the underlying disk access requires word-aligned
 * addresses.
 */

/* how many bytes of valid data in read_buf[] */
static uint32_t read_buf_valid = 0;

/* how many bytes already consumed from read_buf[] */
static uint32_t read_buf_offset = 0;

/* how many bytes already written into page so far */
static uint32_t write_buf_offset = 0;

/* how many bytes valid in page for writing */
static uint32_t write_buf_valid = 0;

/* how many blocks remain to fulfill current SCSI request */
static uint32_t blocks_remaining = 0;

/* starting LBA of the current read */
static uint32_t current_lba = 0;

/* Write operation context for current CBW */
static uint32_t write_lba = 0;            /* initial LBA from CBW */
static uint32_t write_blocks = 0;         /* total blocks requested in CBW */
static uint32_t write_req_length = 0;     /* total bytes requested in CBW */
static uint32_t write_flushed_length = 0; /* bytes flushed to disk so far */

#define READ_AHEAD_BLOCKS 128
#define BLOCK_SIZE        512

static uint8_t __aligned(4) page[READ_AHEAD_BLOCKS * BLOCK_SIZE + CONFIG_MASS_STORAGE_BULK_EP_MPS];

#define MAX_PACKET  READ_AHEAD_BLOCKS * BLOCK_SIZE //512//CONFIG_MASS_STORAGE_BULK_EP_MPS
//#define MAX_PACKET    CONFIG_MASS_STORAGE_BULK_EP_MPS

/* Initialized during mass_storage_init() */
static uint32_t memory_size;
static uint32_t block_count;
static const char *disk_pdrv = CONFIG_MASS_STORAGE_DISK_NAME;

#define MSD_OUT_EP_IDX			0
#define MSD_IN_EP_IDX			1

static void mass_storage_bulk_out(uint8_t ep,
				  enum usb_dc_ep_cb_status_code ep_status);
static void mass_storage_bulk_in(uint8_t ep,
				 enum usb_dc_ep_cb_status_code ep_status);

/* Describe EndPoints configuration */
static struct usb_ep_cfg_data mass_ep_data[] = {
	{
		.ep_cb	= mass_storage_bulk_out,
		.ep_addr = MASS_STORAGE_OUT_EP_ADDR
	},
	{
		.ep_cb = mass_storage_bulk_in,
		.ep_addr = MASS_STORAGE_IN_EP_ADDR
	}
};

/* CSW Status */
enum Status {
	CSW_PASSED,
	CSW_FAILED,
	CSW_ERROR,
};

/* MSC Bulk-only Stage */
enum Stage {
	MSC_READ_CBW,     /* wait a CBW */
	MSC_ERROR,        /* error */
	MSC_PROCESS_CBW,  /* process a CBW request */
	MSC_SEND_CSW,     /* send a CSW */
	MSC_WAIT_CSW      /* wait that a CSW has been effectively sent */
};

/* state of the bulk-only state machine */
static enum Stage stage;

/*current CBW*/
static struct CBW cbw;

/*CSW which will be sent*/
static struct CSW csw;

/*addr where will be read or written data*/
static uint32_t addr;

/*length of a reading or writing*/
static uint32_t length;

static uint8_t max_lun_count;

/*memory OK (after a memoryVerify)*/
static bool memOK;

#define INQ_VENDOR_ID_LEN 8
#define INQ_PRODUCT_ID_LEN 16
#define INQ_REVISION_LEN 4

struct dabc_inquiry_data {
	uint8_t head[8];
	uint8_t t10_vid[INQ_VENDOR_ID_LEN];
	uint8_t product_id[INQ_PRODUCT_ID_LEN];
	uint8_t product_rev[INQ_REVISION_LEN];
} __packed;

static const struct dabc_inquiry_data inq_rsp = {
	.head = {0x00, 0x80, 0x00, 0x01, 36 - 4, 0x80, 0x00, 0x00},
	.t10_vid = CONFIG_MASS_STORAGE_INQ_VENDOR_ID,
	.product_id = CONFIG_MASS_STORAGE_INQ_PRODUCT_ID,
	.product_rev = CONFIG_MASS_STORAGE_INQ_REVISION,
};

BUILD_ASSERT(sizeof(CONFIG_MASS_STORAGE_INQ_VENDOR_ID) == (INQ_VENDOR_ID_LEN + 1),
	"CONFIG_MASS_STORAGE_INQ_VENDOR_ID must be 8 characters (pad with spaces)");

BUILD_ASSERT(sizeof(CONFIG_MASS_STORAGE_INQ_PRODUCT_ID) == (INQ_PRODUCT_ID_LEN + 1),
	"CONFIG_MASS_STORAGE_INQ_PRODUCT_ID must be 16 characters (pad with spaces)");

BUILD_ASSERT(sizeof(CONFIG_MASS_STORAGE_INQ_REVISION) == (INQ_REVISION_LEN + 1),
	"CONFIG_MASS_STORAGE_INQ_REVISION must be 4 characters (pad with spaces)");

static void msd_state_machine_reset(void)
{

    stage = MSC_READ_CBW;
    blocks_remaining = 0;
    current_lba = 0;
    read_buf_valid = 0;
    read_buf_offset = 0;
    write_buf_offset = 0;
    write_buf_valid = 0;
    write_lba = 0;
    write_blocks = 0;
    write_req_length = 0;
    write_flushed_length = 0;

}

static void msd_init(void)
{
	(void)memset((void *)&cbw, 0, sizeof(struct CBW));
	(void)memset((void *)&csw, 0, sizeof(struct CSW));
	(void)memset(page, 0, sizeof(page));
	addr = 0U;
	length = 0U;
}

static void sendCSW(void)
{
	csw.Signature = CSW_Signature;
	if (usb_write(mass_ep_data[MSD_IN_EP_IDX].ep_addr, (uint8_t *)&csw,
		      sizeof(struct CSW), NULL) != 0) {
		LOG_ERR("usb write failure");
	}
	stage = MSC_WAIT_CSW;
}

static bool write(uint8_t *buf, uint16_t size)
{
	if (size >= cbw.DataLength) {
		size = cbw.DataLength;
	}

	/* updating the State Machine , so that we send CSW when this
	 * transfer is complete, ie when we get a bulk in callback
	 */
	stage = MSC_SEND_CSW;

	if (usb_write(mass_ep_data[MSD_IN_EP_IDX].ep_addr, buf, size, NULL)) {
		LOG_ERR("USB write failed");
		return false;
	}

	csw.DataResidue -= size;
	csw.Status = CSW_PASSED;
	return true;
}

/**
 * @brief Handler called for Class requests not handled by the USB stack.
 *
 * @param pSetup    Information about the request to execute.
 * @param len       Size of the buffer.
 * @param data      Buffer containing the request result.
 *
 * @return  0 on success, negative errno code on fail.
 */
static int mass_storage_class_handle_req(struct usb_setup_packet *setup,
					 int32_t *len, uint8_t **data)
{
	if (setup->wIndex != mass_cfg.if0.bInterfaceNumber ||
	    setup->wValue != 0) {
		LOG_ERR("Invalid setup parameters");
		return -EINVAL;
	}

	if (usb_reqtype_is_to_device(setup)) {
		if (setup->bRequest == MSC_REQUEST_RESET &&
		    setup->wLength == 0) {
			LOG_DBG("MSC_REQUEST_RESET");
			msd_state_machine_reset();
			return 0;
		}
	} else {
		if (setup->bRequest == MSC_REQUEST_GET_MAX_LUN &&
		    setup->wLength == 1) {
			LOG_DBG("MSC_REQUEST_GET_MAX_LUN");
			max_lun_count = 0U;
			*data = (uint8_t *)(&max_lun_count);
			*len = 1;
			return 0;
		}
	}

	LOG_WRN("Unsupported bmRequestType 0x%02x bRequest 0x%02x",
		setup->bmRequestType, setup->bRequest);
	return -ENOTSUP;
}

static void testUnitReady(void)
{
	if (cbw.DataLength != 0U) {
		if ((cbw.Flags & 0x80) != 0U) {
			LOG_WRN("Stall IN endpoint");
			usb_ep_set_stall(mass_ep_data[MSD_IN_EP_IDX].ep_addr);
		} else {
			LOG_WRN("Stall OUT endpoint");
			usb_ep_set_stall(mass_ep_data[MSD_OUT_EP_IDX].ep_addr);
		}
	}

	csw.Status = CSW_PASSED;
	sendCSW();
}

static bool requestSense(void)
{
	uint8_t request_sense[] = {
		0x70,
		0x00,
		0x05,   /* Sense Key: illegal request */
		0x00,
		0x00,
		0x00,
		0x00,
		0x0A,
		0x00,
		0x00,
		0x00,
		0x00,
		0x30,
		0x01,
		0x00,
		0x00,
		0x00,
		0x00,
	};

	return write(request_sense, sizeof(request_sense));
}

static bool inquiryRequest(void)
{
	return write((uint8_t *)&inq_rsp, sizeof(inq_rsp));
}

static bool modeSense6(void)
{
	uint8_t sense6[] = { 0x03, 0x00, 0x00, 0x00 };

	return write(sense6, sizeof(sense6));
}

static bool readFormatCapacity(void)
{
	uint8_t capacity[] = { 0x00, 0x00, 0x00, 0x08,
			(uint8_t)((block_count >> 24) & 0xff),
			(uint8_t)((block_count >> 16) & 0xff),
			(uint8_t)((block_count >> 8) & 0xff),
			(uint8_t)((block_count >> 0) & 0xff),

			0x02,
			(uint8_t)((BLOCK_SIZE >> 16) & 0xff),
			(uint8_t)((BLOCK_SIZE >> 8) & 0xff),
			(uint8_t)((BLOCK_SIZE >> 0) & 0xff),
	};

	return write(capacity, sizeof(capacity));
}

static bool readCapacity(void)
{
	uint8_t capacity[8];

	sys_put_be32(block_count - 1, &capacity[0]);
	sys_put_be32(BLOCK_SIZE, &capacity[4]);

	return write(capacity, sizeof(capacity));
}

static void thread_memory_read_done(void)
{
	uint32_t n;

    /* Check if read_buf actually has data */
    if (read_buf_offset >= read_buf_valid) {
        /* Shouldn't happen, but safe-guard */
        if (blocks_remaining > 0) {
            thread_op = THREAD_OP_READ_QUEUED;
            k_sem_give(&disk_wait_sem);
            return;
        } else {
            csw.Status = CSW_PASSED;
            stage = MSC_SEND_CSW;
            return;
        }
    }

    /* How many bytes shall we send in this USB packet? */
    n = (length > MAX_PACKET) ? MAX_PACKET : length;

    uint32_t remaining_in_buf = read_buf_valid - read_buf_offset;
    if (n > remaining_in_buf) {
        n = remaining_in_buf;
    }

    if ((addr + n) > memory_size) {
        n = memory_size - addr;
        stage = MSC_ERROR;
    }

    if (usb_write(mass_ep_data[MSD_IN_EP_IDX].ep_addr,
            &page[read_buf_offset], n, NULL) != 0) {
        LOG_ERR("Failed to write EP 0x%x",
            mass_ep_data[MSD_IN_EP_IDX].ep_addr);
    }

    read_buf_offset += n;
    addr += n;
    length -= n;
    csw.DataResidue -= n;

    if (!length || (stage != MSC_PROCESS_CBW)) {
        csw.Status = (stage == MSC_PROCESS_CBW) ?
            CSW_PASSED : CSW_FAILED;
        stage = (stage == MSC_PROCESS_CBW) ? MSC_SEND_CSW : stage;
    }
}


static void memoryRead(void)
{
    uint32_t n;

    /* Check if read_buf is empty */
    if (read_buf_offset >= read_buf_valid) {
        if (blocks_remaining > 0) {
            /* Buffer empty, but more blocks remain. Trigger disk read. */
            thread_op = THREAD_OP_READ_QUEUED;
            LOG_DBG("Read-ahead buffer empty, refilling...");
            k_sem_give(&disk_wait_sem);
            return;
        } else {
            /* All blocks read and sent, we're done. */
            csw.Status = CSW_PASSED;
            stage = MSC_SEND_CSW;
            return;
        }
    }

    /* Determine how many bytes to send this time */
    n = (length > MAX_PACKET) ? MAX_PACKET : length;

    /* Ensure we don't read past the buffer's valid data */
    uint32_t remaining_in_buf = read_buf_valid - read_buf_offset;
    if (n > remaining_in_buf) {
        n = remaining_in_buf;
    }

    /* Bounds check against total memory size (safety net) */
    if ((addr + n) > memory_size) {
        n = memory_size - addr;
        stage = MSC_ERROR;
    }

    /* Write data to USB endpoint from read_buf */
    if (usb_write(
            mass_ep_data[MSD_IN_EP_IDX].ep_addr,
            &page[read_buf_offset],
            n,
            NULL) != 0) {
        LOG_ERR("Failed to write EP 0x%x",
            mass_ep_data[MSD_IN_EP_IDX].ep_addr);
    }

    read_buf_offset += n;
	addr += n;
	length -= n;

	csw.DataResidue -= n;

	if (!length || (stage != MSC_PROCESS_CBW)) {
		csw.Status = (stage == MSC_PROCESS_CBW) ?
			CSW_PASSED : CSW_FAILED;
		stage = (stage == MSC_PROCESS_CBW) ? MSC_SEND_CSW : stage;
	}
}

static bool check_cbw_data_length(void)
{
	if (!cbw.DataLength) {
		LOG_WRN("Zero length in CBW");
		csw.Status = CSW_FAILED;
		sendCSW();
		return false;
	}

	return true;
}

static bool infoTransfer(void)
{
	uint32_t n;

	if (!check_cbw_data_length()) {
		return false;
	}

	/* Logical Block Address of First Block */
	n = sys_get_be32(&cbw.CB[2]);

	LOG_DBG("LBA (block) : 0x%x ", n);
	if ((n * BLOCK_SIZE) >= memory_size) {
		LOG_ERR("LBA out of range");
		csw.Status = CSW_FAILED;
		sendCSW();
		return false;
	}

	addr = n * BLOCK_SIZE;

	/* Number of Blocks to transfer */
	switch (cbw.CB[0]) {
	case READ10:
	case WRITE10:
	case VERIFY10:
		n = sys_get_be16(&cbw.CB[7]);
		break;

	case READ12:
	case WRITE12:
		n = sys_get_be32(&cbw.CB[6]);
		break;
	}

	LOG_DBG("Size (block) : 0x%x ", n);
	length = n * BLOCK_SIZE;

	if (cbw.DataLength != length) {
		if ((cbw.Flags & 0x80) != 0U) {
			LOG_WRN("Stall IN endpoint");
			usb_ep_set_stall(mass_ep_data[MSD_IN_EP_IDX].ep_addr);
		} else {
			LOG_WRN("Stall OUT endpoint");
			usb_ep_set_stall(mass_ep_data[MSD_OUT_EP_IDX].ep_addr);
		}

		csw.Status = CSW_FAILED;
		sendCSW();
		return false;
	}

	return true;
}

static void fail(void)
{
	if (cbw.DataLength) {
		/* Stall data stage */
		usb_ep_set_stall(mass_ep_data[MSD_IN_EP_IDX].ep_addr);
	}

	csw.Status = CSW_FAILED;
	sendCSW();
}

static void CBWDecode(uint8_t *buf, uint16_t size)
{
	if (size != sizeof(cbw)) {
		LOG_ERR("size != sizeof(cbw)");
		return;
	}

	memcpy((uint8_t *)&cbw, buf, size);
	if (cbw.Signature != CBW_Signature) {
		LOG_ERR("CBW Signature Mismatch");
		return;
	}

	csw.Tag = cbw.Tag;
	csw.DataResidue = cbw.DataLength;

	if ((cbw.CBLength <  1) || (cbw.CBLength > 16) || (cbw.LUN != 0U)) {
		LOG_WRN("cbw.CBLength %d", cbw.CBLength);
		fail();
	} else {
		switch (cbw.CB[0]) {
		case TEST_UNIT_READY:
			LOG_DBG(">> TUR");
			testUnitReady();
			break;
		case REQUEST_SENSE:
			LOG_DBG(">> REQ_SENSE");
			if (check_cbw_data_length()) {
				requestSense();
			}
			break;
		case INQUIRY:
			LOG_DBG(">> INQ");
			if (check_cbw_data_length()) {
				inquiryRequest();
			}
			break;
		case MODE_SENSE6:
			LOG_DBG(">> MODE_SENSE6");
			if (check_cbw_data_length()) {
				modeSense6();
			}
			break;
		case READ_FORMAT_CAPACITIES:
			LOG_DBG(">> READ_FORMAT_CAPACITY");
			if (check_cbw_data_length()) {
				readFormatCapacity();
			}
			break;
		case READ_CAPACITY:
			LOG_DBG(">> READ_CAPACITY");
			if (check_cbw_data_length()) {
				readCapacity();
			}
			break;
		case READ10:
		case READ12:
			LOG_DBG(">> READ");

            uint32_t lba = sys_get_be32(&cbw.CB[2]);
            uint32_t blocks;
            uint32_t req_length;

            if (cbw.CB[0] == READ10) {
                blocks = sys_get_be16(&cbw.CB[7]);
            } else {
                blocks = sys_get_be32(&cbw.CB[6]);
            }

            req_length = blocks * BLOCK_SIZE;

            LOG_INF("Host READ request: LBA=%u, Blocks=%u, Bytes=%u",
                lba, blocks, req_length);

            /* Initialize read-ahead state for this SCSI command */
            blocks_remaining = sys_get_be16(&cbw.CB[7]);
            current_lba = lba;

            read_buf_valid = 0;
            read_buf_offset = 0;


			if (infoTransfer()) {
				if ((cbw.Flags & 0x80)) {
					stage = MSC_PROCESS_CBW;
					memoryRead();
				} else {
					usb_ep_set_stall(
					  mass_ep_data[MSD_OUT_EP_IDX].ep_addr);
					LOG_WRN("Stall OUT endpoint");
					csw.Status = CSW_ERROR;
					sendCSW();
				}
			}
			break;
		case WRITE10:
		case WRITE12:
			LOG_DBG(">> WRITE");

            if (cbw.CB[0] == WRITE10) {
                lba = sys_get_be32(&cbw.CB[2]);
                blocks = sys_get_be16(&cbw.CB[7]);
            } else {
                lba = sys_get_be32(&cbw.CB[2]);
                blocks = sys_get_be32(&cbw.CB[6]);
            }

            req_length = blocks * BLOCK_SIZE;

            LOG_INF("Host WRITE request: LBA=%u, Blocks=%u, Bytes=%u",
                    lba, blocks, req_length);

            write_lba = lba;
            write_blocks = blocks;
            write_req_length = req_length;
            write_flushed_length = 0;

			if (infoTransfer()) {
				if (!(cbw.Flags & 0x80)) {
					stage = MSC_PROCESS_CBW;
				} else {
					usb_ep_set_stall(
					  mass_ep_data[MSD_IN_EP_IDX].ep_addr);
					LOG_WRN("Stall IN endpoint");
					csw.Status = CSW_ERROR;
					sendCSW();
				}
			}
			break;
		case VERIFY10:
			LOG_DBG(">> VERIFY10");
			if (!(cbw.CB[1] & 0x02)) {
				csw.Status = CSW_PASSED;
				sendCSW();
				break;
			}
			if (infoTransfer()) {
				if (!(cbw.Flags & 0x80)) {
					stage = MSC_PROCESS_CBW;
					memOK = true;
				} else {
					usb_ep_set_stall(
					  mass_ep_data[MSD_IN_EP_IDX].ep_addr);
					LOG_WRN("Stall IN endpoint");
					csw.Status = CSW_ERROR;
					sendCSW();
				}
			}
			break;
		case MEDIA_REMOVAL:
			LOG_DBG(">> MEDIA_REMOVAL");
			csw.Status = CSW_PASSED;
			sendCSW();
			break;
		default:
			LOG_WRN(">> default CB[0] %x", cbw.CB[0]);
			fail();
			break;
		}		/*switch(cbw.CB[0])*/
	}		/* else */

}

static void memoryVerify(uint8_t *buf, uint16_t size)
{
	uint32_t n;

	if ((addr + size) > memory_size) {
		size = memory_size - addr;
		stage = MSC_ERROR;
		usb_ep_set_stall(mass_ep_data[MSD_OUT_EP_IDX].ep_addr);
		LOG_WRN("Stall OUT endpoint");
	}

	/* beginning of a new block -> load a whole block in RAM */
	if (!(addr % BLOCK_SIZE)) {
		LOG_DBG("Disk READ sector %d", addr/BLOCK_SIZE);
		if (disk_access_read(disk_pdrv, page, addr/BLOCK_SIZE, 1)) {
			LOG_ERR("---- Disk Read Error %d", addr/BLOCK_SIZE);
		}
	}

	/* info are in RAM -> no need to re-read memory */
	for (n = 0U; n < size; n++) {
		if (page[addr%BLOCK_SIZE + n] != buf[n]) {
			LOG_DBG("Mismatch sector %d offset %d",
				addr/BLOCK_SIZE, n);
			memOK = false;
			break;
		}
	}

	addr += size;
	length -= size;
	csw.DataResidue -= size;

	if (!length || (stage != MSC_PROCESS_CBW)) {
		csw.Status = (memOK && (stage == MSC_PROCESS_CBW)) ?
						CSW_PASSED : CSW_FAILED;
		sendCSW();
	}
}

static void memoryWrite(uint8_t *buf, uint16_t size)
{
    if (write_buf_offset + size > sizeof(page)) {
        LOG_ERR("Write buffer overflow!");
        stage = MSC_ERROR;
        usb_ep_set_stall(mass_ep_data[MSD_OUT_EP_IDX].ep_addr);
        sendCSW();
        return;
    }

    memcpy(&page[write_buf_offset], buf, size);
    write_buf_offset += size;

    length -= size;
    csw.DataResidue -= size;

    if ((write_buf_offset >= sizeof(page)) || (length == 0)) {
        write_buf_valid = write_buf_offset;

        thread_op = THREAD_OP_WRITE_QUEUED;
        k_sem_give(&disk_wait_sem);
        return;
    }

    usb_ep_read_continue(mass_ep_data[MSD_OUT_EP_IDX].ep_addr);
}


static void mass_storage_bulk_out(uint8_t ep,
		enum usb_dc_ep_cb_status_code ep_status)
{
	uint32_t bytes_read = 0U;
	uint8_t bo_buf[CONFIG_MASS_STORAGE_BULK_EP_MPS];

	ARG_UNUSED(ep_status);

	usb_ep_read_wait(ep, bo_buf, CONFIG_MASS_STORAGE_BULK_EP_MPS,
			 &bytes_read);

	switch (stage) {
	/*the device has to decode the CBW received*/
	case MSC_READ_CBW:
		LOG_DBG("> BO - MSC_READ_CBW");
		CBWDecode(bo_buf, bytes_read);
		break;

	/*the device has to receive data from the host*/
	case MSC_PROCESS_CBW:
		switch (cbw.CB[0]) {
		case WRITE10:
		case WRITE12:
			/* LOG_DBG("> BO - PROC_CBW WR");*/
			memoryWrite(bo_buf, bytes_read);
			break;
		case VERIFY10:
			LOG_DBG("> BO - PROC_CBW VER");
			memoryVerify(bo_buf, bytes_read);
			break;
		default:
			LOG_ERR("> BO - PROC_CBW default <<ERROR!!!>>");
			break;
		}
		break;

	/*an error has occurred: stall endpoint and send CSW*/
	default:
		LOG_WRN("Stall OUT endpoint, stage: %d", stage);
		usb_ep_set_stall(ep);
		csw.Status = CSW_ERROR;
		sendCSW();
		break;
	}

	if (thread_op != THREAD_OP_WRITE_QUEUED) {
		usb_ep_read_continue(ep);
	} else {
		LOG_DBG("> BO not clearing NAKs yet");
	}

}

static void thread_memory_write_done(void)
{

	if ((!length) || (stage != MSC_PROCESS_CBW)) {
		csw.Status = (stage == MSC_ERROR) ? CSW_FAILED : CSW_PASSED;
		sendCSW();
	}

	thread_op = THREAD_OP_WRITE_DONE;

	usb_ep_read_continue(mass_ep_data[MSD_OUT_EP_IDX].ep_addr);
}

/**
 * @brief EP Bulk IN handler, used to send data to the Host
 *
 * @param ep        Endpoint address.
 * @param ep_status Endpoint status code.
 *
 * @return  N/A.
 */
static void mass_storage_bulk_in(uint8_t ep,
				 enum usb_dc_ep_cb_status_code ep_status)
{
	ARG_UNUSED(ep_status);
	ARG_UNUSED(ep);

	switch (stage) {
	/*the device has to send data to the host*/
	case MSC_PROCESS_CBW:
		switch (cbw.CB[0]) {
		case READ10:
		case READ12:
			/* LOG_DBG("< BI - PROC_CBW  READ"); */
			memoryRead();
			break;
		default:
			LOG_ERR("< BI-PROC_CBW default <<ERROR!!>>");
			break;
		}
		break;

	/*the device has to send a CSW*/
	case MSC_SEND_CSW:
		LOG_DBG("< BI - MSC_SEND_CSW");
		sendCSW();
		break;

	/*the host has received the CSW -> we wait a CBW*/
	case MSC_WAIT_CSW:
		LOG_DBG("< BI - MSC_WAIT_CSW");
		stage = MSC_READ_CBW;
		break;

	/*an error has occurred*/
	default:
		LOG_WRN("Stall IN endpoint, stage: %d", stage);
		usb_ep_set_stall(mass_ep_data[MSD_IN_EP_IDX].ep_addr);
		sendCSW();
		break;
	}
}



/**
 * @brief Callback used to know the USB connection status
 *
 * @param status USB device status code.
 *
 * @return  N/A.
 */
static void mass_storage_status_cb(struct usb_cfg_data *cfg,
				   enum usb_dc_status_code status,
				   const uint8_t *param)
{
	ARG_UNUSED(param);
	ARG_UNUSED(cfg);

	/* Check the USB status and do needed action if required */
	switch (status) {
	case USB_DC_ERROR:
		LOG_DBG("USB device error");
		break;
	case USB_DC_RESET:
		LOG_DBG("USB device reset detected");
		msd_state_machine_reset();
		msd_init();
		break;
	case USB_DC_CONNECTED:
		LOG_DBG("USB device connected");
		break;
	case USB_DC_CONFIGURED:
		LOG_DBG("USB device configured");
		break;
	case USB_DC_DISCONNECTED:
		LOG_DBG("USB device disconnected");
		break;
	case USB_DC_SUSPEND:
		LOG_DBG("USB device suspended");
		break;
	case USB_DC_RESUME:
		LOG_DBG("USB device resumed");
		break;
	case USB_DC_INTERFACE:
		LOG_DBG("USB interface selected");
		break;
	case USB_DC_SOF:
		break;
	case USB_DC_UNKNOWN:
	default:
		LOG_DBG("USB unknown state");
		break;
	}
}

static void mass_interface_config(struct usb_desc_header *head,
				  uint8_t bInterfaceNumber)
{
	ARG_UNUSED(head);

	mass_cfg.if0.bInterfaceNumber = bInterfaceNumber;
}

/* Configuration of the Mass Storage Device send to the USB Driver */
USBD_DEFINE_CFG_DATA(mass_storage_config) = {
	.usb_device_description = NULL,
	.interface_config = mass_interface_config,
	.interface_descriptor = &mass_cfg.if0,
	.cb_usb_status = mass_storage_status_cb,
	.interface = {
		.class_handler = mass_storage_class_handle_req,
		.custom_handler = NULL,
	},
	.num_endpoints = ARRAY_SIZE(mass_ep_data),
	.endpoint = mass_ep_data
};

static void mass_thread_main(int arg1, int unused)
{
	ARG_UNUSED(unused);
	ARG_UNUSED(arg1);

    uint32_t t0, t1, us;
    uint32_t kb_per_sec;

	while (1) {
		k_sem_take(&disk_wait_sem, K_FOREVER);
		LOG_DBG("sem %d", thread_op);

		switch (thread_op) {
		case THREAD_OP_READ_QUEUED:

            /* Save current priority */
            int old_prio = k_thread_priority_get(k_current_get());

            /* Raise priority as high as desired, e.g. -10 */
            k_thread_priority_set(k_current_get(), -10);



            t0 = k_cycle_get_32();

            /* how many blocks can we read into the buffer? */
            uint32_t max_blocks = READ_AHEAD_BLOCKS;

            uint32_t blocks_to_read = (blocks_remaining > max_blocks) ?
                                        max_blocks : blocks_remaining;

            /* Safety check in case blocks_remaining is zero */
            if (blocks_to_read == 0) {
                LOG_WRN("THREAD_OP_READ_QUEUED called but no blocks remaining!");
                thread_memory_read_done();
                break;
            }

            if (disk_access_read(disk_pdrv,
                    page,
                    current_lba,
                    blocks_to_read)) {
                LOG_ERR("!! Disk Read Error at LBA %u", current_lba);
                // Optionally set stage = MSC_ERROR and send CSW
            }

            read_buf_valid = blocks_to_read * BLOCK_SIZE;
            read_buf_offset = 0;

            current_lba += blocks_to_read;
            blocks_remaining -= blocks_to_read;

            t1 = k_cycle_get_32();
            us = k_cyc_to_us_floor32(t1 - t0);

            uint32_t bytes_read = read_buf_valid;

            kb_per_sec = (bytes_read * 1000 / us);

            LOG_INF("Disk read %u blocks (%u bytes) from LBA %u took %u us - %u KB/s",
                blocks_to_read,
                bytes_read,
                current_lba - blocks_to_read,
                us,
                kb_per_sec);

            k_thread_priority_set(k_current_get(), old_prio);

			thread_memory_read_done();
			break;
		case THREAD_OP_WRITE_QUEUED:

            t0 = k_cycle_get_32();

            /* Determine the block offset relative to the original LBA */
            uint32_t block_offset = write_flushed_length / BLOCK_SIZE;
            uint32_t lba_to_write = write_lba + block_offset;

            uint32_t blocks_to_write = write_buf_valid / BLOCK_SIZE;

            if (blocks_to_write > 0) {
                if (disk_access_write(disk_pdrv,
                                    page,
                                    lba_to_write,
                                    blocks_to_write)) {
                    LOG_ERR("Disk Write Error at LBA %u", lba_to_write);
                    stage = MSC_ERROR;
                    csw.Status = CSW_FAILED;
                    sendCSW();
                    break;
                }

                write_flushed_length += blocks_to_write * BLOCK_SIZE;
            }

            /* Check for leftover partial block */
            uint32_t leftover_bytes = write_buf_valid % BLOCK_SIZE;
            if (leftover_bytes > 0) {
                uint32_t leftover_lba = lba_to_write + blocks_to_write;
                uint8_t sector_buf[BLOCK_SIZE] = {0};

                if (disk_access_read(disk_pdrv, sector_buf, leftover_lba, 1)) {
                    LOG_ERR("Disk read before partial write failed!");
                    stage = MSC_ERROR;
                    csw.Status = CSW_FAILED;
                    sendCSW();
                    break;
                }

                memcpy(sector_buf,
                    &page[blocks_to_write * BLOCK_SIZE],
                    leftover_bytes);

                if (disk_access_write(disk_pdrv, sector_buf, leftover_lba, 1)) {
                    LOG_ERR("Disk write for leftover failed!");
                    stage = MSC_ERROR;
                    csw.Status = CSW_FAILED;
                    sendCSW();
                    break;
                }

                write_flushed_length += leftover_bytes;
            }

            t1 = k_cycle_get_32();
            us = k_cyc_to_us_floor32(t1 - t0);

            LOG_INF("Disk write %u bytes at LBA %u took %u us (%u KB/s)",
                    write_buf_valid,
                    lba_to_write,
                    us,
                    (write_buf_valid * 1000) / us);

            write_buf_valid = 0;
            write_buf_offset = 0;

            if (length == 0) {
                LOG_INF("!! Flushing !!");
                if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_CTRL_SYNC, NULL)) {
                    LOG_INF("!! Disk cache sync error !!");
                }
            }

            thread_memory_write_done();
            break;

        default:
            LOG_ERR("XXXXXX thread_op  %d ! XXXXX", thread_op);
        }
    }

}

/**
 * @brief Initialize USB mass storage setup
 *
 * This routine is called to reset the USB device controller chip to a
 * quiescent state. Also it initializes the backing storage and initializes
 * the mass storage protocol state.
 *
 * @param dev device struct.
 *
 * @return negative errno code on fatal failure, 0 otherwise
 */
static int mass_storage_init(const struct device *dev)
{
	uint32_t block_size = 0U;

	ARG_UNUSED(dev);

	if (disk_access_init(disk_pdrv) != 0) {
		LOG_ERR("Storage init ERROR !!!! - Aborting USB init");
		return 0;
	}

	if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
		LOG_ERR("Unable to get sector count - Aborting USB init");
		return 0;
	}

	if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
		LOG_ERR("Unable to get sector size - Aborting USB init");
		return 0;
	}

	if (block_size != BLOCK_SIZE) {
		LOG_ERR("Block Size reported by the storage side is "
			"different from Mass Storage Class page Buffer - "
			"Aborting");
		return 0;
	}


	LOG_INF("Sect Count %d", block_count);
	memory_size = block_count * BLOCK_SIZE;
	LOG_INF("Memory Size %d", memory_size);

	msd_state_machine_reset();
	msd_init();

	k_sem_init(&disk_wait_sem, 0, 1);

	/* Start a thread to offload disk ops */
	k_thread_create(&mass_thread_data, mass_thread_stack,
			CONFIG_MASS_STORAGE_STACK_SIZE,
			(k_thread_entry_t)mass_thread_main, NULL, NULL, NULL,
			DISK_THREAD_PRIO, 0, K_NO_WAIT);

	k_thread_name_set(&mass_thread_data, "usb_mass");

	return 0;
}

SYS_INIT(mass_storage_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
