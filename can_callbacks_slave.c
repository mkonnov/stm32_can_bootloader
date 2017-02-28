#include "common.h"
#include "com_proto.h"
#include "ring_buf.h"
#include "gpio.h"
#include "utils.h"
#include "nv_data.h"
#include "flash.h"
#include "partitions.h"
#include "tinystdio.h"
#include "hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


static bool com_proto_err_flag = 0;

#define CHUNK_SIZE 512

static uint32_t data_flash_offset = 0;
static bool last_data_processed = false;

//SemaphoreHandle_t data_mutex;

void com_proto_dbg(void) {
	static uint8_t led_state;
	gpio_set(CAN_DBG_LED,led_state = ~led_state);
}


static uint8_t data_block[512] = {[0 ... 511] = 0};
static uint16_t data_block_ptr = 0;

void cmd_mode_change(void) {
	register uint32_t id = com_proto_generate_id(0, 2, CMD_MODE_CHANGE_RSP);
	DBG printf("[com_proto]: CMD_MODE_CHANGE_RQ\n");
	if (cookie_flag_set(COOKIE_UPD_FLAG)) {
		com_proto_tx_packet(id, 0, 0);
	}
}

void cmd_reboot(void) {
	register uint32_t id = com_proto_generate_id(0, 2, CMD_MODE_REBOOT_RSP);
	DBG printf("[com_proto]: CMD_MODE_REBOOT_RQ\n");
//	com_proto_tx_packet(CMD_MODE_REBOOT_RSP, 0, 0);
	com_proto_tx_packet(id, 0, 0);
	reset();
}

void cmd_reboot_verify(void) {
	DBG printf("[com_proto]: CMD_MODE_VERIFY_RQ\n");
	com_proto_tx_packet(CMD_MODE_VERIFY_RSP, 0, 0);
}

void cmd_data_start(void) {
	DBG printf("[com_proto]: CMD_DATA_START_RQ\n");
	com_proto_tx_packet(CMD_DATA_START_READY, 0, 0);
}

void update_preroutine(void) {
	DBG printf("[com_proto]: CMD_MODE_UPDATE_START_RQ\n");
	partition_erase(FW_PARTITION);
	com_proto_tx_packet(CMD_MODE_UPDATE_START_RSP, 0, 0);
}

void update_postroutine(void) {
	int fw_partition_addr = partition_get_origin(FW_PARTITION);
	DBG printf("[com_proto]: CMD_DATA_FINISH_RQ\n");
	com_proto_tx_packet(CMD_DATA_FINISH_RSP, 0, 0);
	flash_lock();
	cookie_flag_clear(COOKIE_UPD_FLAG);
	reset();
}


static void cmd_process_data(void) {
	uint8_t data[8];
	if (com_proto_get_rx_data(data)) {
		memcpy(data_block + data_block_ptr, data, 8);
		data_block_ptr += 8;
		if (data_block_ptr == CHUNK_SIZE) {
			flash_write_block(partition_get_origin(FW_PARTITION) + data_flash_offset, data_block, CHUNK_SIZE);
			data_block_ptr = 0;
			data_flash_offset += CHUNK_SIZE;
			last_data_processed = true;
		}
		com_proto_dbg();
	}
}

static void cmd_process_chunk(void) {

}
static void cmd_process_len(void) {
	uint8_t data[8];
	uint32_t len;
	if (com_proto_get_rx_data(data)) {
		len = *(uint32_t*)data;
		DBG printf("[com_proto]: image size %d\n", len);
		flash_unlock();
	}
}

static void cmd_process_block_ack(void) {
	if (last_data_processed) {
		com_proto_tx_packet(CMD_DATA_BLK_ACK_RSP, 0, 0);
	}
	last_data_processed = false;
}

static void cmd_process_flash_read(void) {
	uint8_t data[8];
	int32_t address, length, offset;
	uint8_t tx_data[8];
	offset = 0;
	if (com_proto_get_rx_data(data)) {
		address = *(uint32_t*)data;
		length = *(uint32_t*)(data + 4);
		while (offset < length) {
			memcpy(tx_data, address + offset, 8);
			com_proto_tx_packet(CMD_DATA_FLASH_READ_RSP, tx_data, 8);
			offset += 8;
		}
	}
}

void cmd_rollback(void) {
#ifdef HARDWARE_F303
	gpio_set(LD5, 1);
#endif
	partition_copy(FW_PARTITION, FW_BACKUP_PARTITION);
	com_proto_tx_packet(CMD_MODE_ROLLBACK_RSP, 0, 0);
	cookie_flag_clear(COOKIE_UPD_FLAG);
	reset();

#ifdef HARDWARE_F303
	gpio_set(LD5, 0);
	gpio_set(LD10, 0);
#endif
}

void cmd_responce_data_size_and_addr(void) {
	uint8_t data[8];
	uint32_t size = partition_get_size(FW_PARTITION);
	uint32_t addr = partition_get_origin(FW_PARTITION);
	memcpy(data, &size, 4);
	memcpy(data + 4, &addr, 4);
	com_proto_tx_packet(CMD_REQUEST_DATA_SIZE_AND_ADDR_RSP, data, 8);
}

com_proto_callback_t slave_iap_callbacks[] = {
	{cmd_mode_change, CMD_MODE_CHANGE_RQ},
	{cmd_reboot, CMD_MODE_REBOOT_RQ},
	{cmd_reboot_verify, CMD_MODE_VERIFY_RQ},
	{update_preroutine, CMD_MODE_UPDATE_START_RQ},
	{cmd_data_start, CMD_DATA_START_RQ},
	{cmd_process_len, CMD_DATA_LEN},
	{cmd_process_data, CMD_DATA_BLK_CHUNK},
	{cmd_process_block_ack, CMD_DATA_BLK_ACK_RQ},
	{cmd_process_flash_read, CMD_DATA_FLASH_READ},
	{update_postroutine, CMD_DATA_FINISH_RQ},
	{cmd_rollback,	CMD_MODE_ROLLBACK},
	{cmd_responce_data_size_and_addr, CMD_REQUEST_DATA_SIZE_AND_ADDR},
	{NULL, NULL},
};

com_proto_callback_t slave_normal_mode_callbacks[] = {
	{cmd_mode_change, CMD_MODE_CHANGE_RQ},
	{cmd_reboot, CMD_MODE_REBOOT_RQ},
	{NULL, NULL}
};
