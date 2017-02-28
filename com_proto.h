#ifndef COM_PROTO_H
#define COM_PROTO_H

#include "board.h"

/* protocol definitions */
/* NOTE CAN driver internally AND'ing given ID by mask 0x1fffffff */
/* IDs for slave management commands */

#define CMD_RSP_FLAG		0x1000
#define CMD_MODE_MASK		0xffff			/* mask of MODE IDs */
#define CMD_MODE_CHANGE_RQ 	0x0001			/* request to change slave mode */
#define CMD_MODE_CHANGE_RSP	0x0001 | CMD_RSP_FLAG	/* confirmation of change mode getting */
#define CMD_MODE_VERIFY_RQ	0x0003			/* request to get slave mode */
#define CMD_MODE_VERIFY_RSP	0x0003 | CMD_RSP_FLAG	/* response containing the current mode */
#define CMD_MODE_REBOOT_RQ	0x0004			/* reboot request */
#define CMD_MODE_REBOOT_RSP	0x0004 | CMD_RSP_FLAG	/* reboot response */
#define CMD_MODE_UPDATE_START_RQ 0x0005 		/* update start request */
#define CMD_MODE_UPDATE_START_RSP 0x0005 | CMD_RSP_FLAG	/* update start response */
#define CMD_MODE_ROLLBACK	0x0010
#define CMD_MODE_ROLLBACK_RSP	0x0010 | CMD_RSP_FLAG /* commands slave to rollback the data */ 
/* IDs related to data transfer*/
#define CMD_DATA_FLAG		0x2000			/* indicating a data in CAN DATA field */
#define CMD_DATA_START_RQ	0x0006
#define CMD_DATA_START_READY	0x0007
#define CMD_DATA_BLK_CHUNK	0x0008	| CMD_DATA_FLAG	/* data chunk */
#define CMD_DATA_LEN		0x0009	| CMD_DATA_FLAG	/* length of data */
#define CMD_DATA_CRC		0x000a			/* CRC for the last data */
#define CMD_DATA_CRC_OK		0x000b			/* CRC match */
#define CMD_DATA_CRC_ERR	0x000c			/* CRC mismatch */
#define CMD_DATA_BLK_ACK_RQ	0x000d
#define CMD_DATA_BLK_ACK_RSP	0x000d | CMD_RSP_FLAG
#define CMD_DATA_FINISH_RQ	0x000e
#define CMD_DATA_FINISH_RSP	0x000e | CMD_RSP_FLAG
#define CMD_DATA_FLASH_READ	0x000f | CMD_DATA_FLAG
#define CMD_DATA_FLASH_READ_RSP 0x000f | CMD_DATA_FLAG | CMD_RSP_FLAG
#define CMD_REQUEST_DATA_SIZE_AND_ADDR 0x0011
#define CMD_REQUEST_DATA_SIZE_AND_ADDR_RSP 0x0011 | CMD_DATA_FLAG | CMD_RSP_FLAG


#define ID_SRC_MASK	0x007e0000
#define ID_DST_MASK	0x1f800000


#define ID_SLAVE1	0x01000000
#define ID_SLAVE2	0x02000000


#define COM_PROTO_PKT_SIZE	8	/* bytes */
#define COM_PROTO_BLK_SIZE	16	/* packets (max amount to calculate CRC)*/

#define com_proto_generate_id(addr_dst, addr_src, flags) ((addr_dst << 24) | (addr_src << 16) | (flags & CMD_MODE_MASK))
#define com_proto_src_from_id(id) (uint8_t)((id >> 16) & 0xff)
#define com_proto_dst_from_id(id) (uint8_t)((id >> 24) & 0xff)
#define com_proto_flags_from_id(id) (id & CMD_MODE_MASK)

#define COM_PROTO_CALLBACKS_MAX_NUM	64
typedef struct {
	void (*callback)(void);
	uint32_t id;
} com_proto_callback_t;


void com_proto_init(uint8_t current_node_address, com_proto_callback_t *pcallbacks);
uint8_t com_proto_tx_packet(uint32_t id, uint8_t *data, uint8_t len);
uint8_t com_proto_tx_block(uint32_t *data, uint32_t len);
void com_proto_process_id(uint32_t cmd);
void com_proto_process_rx_IRQ(uint32_t id, uint64_t data, uint8_t dlc);
void com_proto_process_tx_IRQ(void);
void com_proto_fifo_init(void);
bool com_proto_get_rx_data(uint8_t *data);
bool com_proto_get_rx_id(uint32_t *id, uint32_t timeout);
bool com_proto_expect_id(uint32_t id_exp, uint32_t timeout);
void com_proto_register_callbacks(com_proto_callback_t* callbacks);
void com_proto_callback_by_id(uint32_t received_id);

#endif /* COM_PROTO_H */
