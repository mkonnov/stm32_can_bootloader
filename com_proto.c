#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "com_proto.h"
#include "can.h"
#include "gpio.h"
#include "ring_buf.h"
#include "crc32hw.h"
#include "crc32.h"
#include "semphr.h"

static SemaphoreHandle_t can_id_semphr = NULL;
static BaseType_t xHigherPriorityTaskWoken;


extern uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage);


static CanTxMsg pkt_tx;
static CanRxMsg pkt_rx;

#define DATA_RBUF_LEN 	128
#define ID_RBUF_LEN	128

rbuf_t data_rbuf, id_rbuf;
uint64_t data_fifo[DATA_RBUF_LEN];
uint32_t id_fifo[ID_RBUF_LEN];


static com_proto_callback_t com_proto_callbacks[64];

static bool com_proto_err_flag = 0;

static uint8_t node_address;

void com_proto_id_task(void) {
	uint32_t id;
	while (true) {
		while (com_proto_get_rx_id(&id, 2000)) {
			com_proto_callback_by_id(id);
		}
		vTaskDelay(50);
	}
}

void com_proto_init(uint8_t current_node_address, com_proto_callback_t *pcallbacks) {
	node_address = current_node_address;
	can_id_semphr = xSemaphoreCreateCounting(256, 0);
	com_proto_fifo_init();
	// If callbacks are presented - then create received id ring buffer
	// polling function. Otherwise it must be polled in place where needed.
	if (pcallbacks != NULL) {
		com_proto_register_callbacks(pcallbacks);
		xTaskCreate( com_proto_id_task, "com_tsk", configMINIMAL_STACK_SIZE * 2,\
			NULL, tskIDLE_PRIORITY + 1, NULL);
	}
}

/* transmits the packet of max 8 byte length */
uint8_t com_proto_tx_packet(uint32_t id, uint8_t *data, uint8_t len) {
	uint8_t res = 0;
	uint8_t tx_status, tx_mailbox;
	uint16_t tx_timeout = 0x0fff;
	pkt_tx.IDE 	= CAN_ID_EXT;
	pkt_tx.RTR   	= CAN_RTR_DATA;
	pkt_tx.DLC   	= len;
	pkt_tx.ExtId	= id;

	memset(pkt_tx.Data, 0, 8);
	if (len) {
		memcpy(pkt_tx.Data, data, len);
	}
	tx_mailbox = CAN_Transmit(CAN1, &pkt_tx);
	do {
		tx_status = can_tx_status(CAN1, tx_mailbox);
	} while (tx_status != CAN_TxStatus_Ok && tx_timeout--);
	//} while (tx_status == CAN_TxStatus_Pending && tx_timeout--);

	if (tx_status == CAN_TxStatus_Ok)
		res = 1;
	return res;
}

/* transmits the block of data */
uint8_t com_proto_tx_block(uint32_t *data, uint32_t len) {
	volatile uint32_t
	blks_to_send,
	pkts_to_send,
	bytes_to_send,
	bytes_sent = 0,
	crc = 0;
	bytes_to_send = len;
	uint8_t res = 1; 
	/* 
	 * amount of 128 byte blocks to be sent 
	 */
	blks_to_send = len / (COM_PROTO_BLK_SIZE * COM_PROTO_PKT_SIZE) + 1;
	while (blks_to_send--) {
		/* 
		 * the last packet, not guaranteed to be full 
		 */
		if (!blks_to_send) {
			/* 
			 * how many packets in the last block 
			 */
			pkts_to_send = (len % (COM_PROTO_BLK_SIZE * COM_PROTO_PKT_SIZE)) / COM_PROTO_PKT_SIZE;
			/* 
			 * if there's some bytes that not fitted in last packet,
			 * then we need to fill it in the next one
			 */
			if (bytes_to_send - pkts_to_send * COM_PROTO_PKT_SIZE)
				pkts_to_send++;
		}
		/* 
		 * not the last, guaranteed to be full
		 */
		else {
			pkts_to_send = COM_PROTO_BLK_SIZE;
		}
		//crc = crc32(crc, data + bytes_sent, pkts_to_send * COM_PROTO_PKT_SIZE);
		res &= com_proto_tx_packet(CMD_DATA_LEN, 0, 0);
		/* 
		 * no need to transfer an array of data if first tx is failed 
		 */
		if (res) {
			for (pkts_to_send; pkts_to_send; pkts_to_send--) {
				//res &= com_proto_tx_packet(CMD_DATA_CHUNK, data + bytes_sent, COM_PROTO_PKT_SIZE);
				bytes_sent += COM_PROTO_PKT_SIZE;
			}
			res &= com_proto_tx_packet(CMD_DATA_CRC, &crc, sizeof(crc));
		}
	}
	return res;
}

void com_proto_fifo_init(void) {
	rbuf64_init(&data_rbuf, data_fifo, DATA_RBUF_LEN);
	rbuf32_init(&id_rbuf, id_fifo, ID_RBUF_LEN);
}

bool com_proto_get_rx_data(uint8_t *data) {
	if (rbuf64_get(&data_rbuf, data))
		return true;
	else
		return false;
}

/* 
 * CAN ID field is always presented in CAN message
 * so this fhunction can be used for message awaiting
 * with a specified timeout.
 */
bool com_proto_get_rx_id(uint32_t *id, uint32_t timeout) {
	if (xSemaphoreTake(can_id_semphr, (TickType_t)timeout) != pdTRUE)
		return false;
	if (rbuf32_get(&id_rbuf, id))
		return true;
	else
		return false;
}

/* await for specified CAN ID */
bool com_proto_expect_id(uint32_t id_exp, uint32_t timeout) {
	uint32_t rx_id;
	bool res = false;
	if (com_proto_get_rx_id(&rx_id, timeout)) {
		if (rx_id == id_exp)
			res = true;
	}
	return res;
}

void com_proto_register_callbacks(com_proto_callback_t* callbacks) {
	char i;
	char size = sizeof(com_proto_callback_t);
	for (i = 0; i < COM_PROTO_CALLBACKS_MAX_NUM; i++) {
		if (callbacks[i].callback != NULL)
			memcpy(&com_proto_callbacks[i], &callbacks[i], size);
		else
			break;
	}
}

void com_proto_callback_by_id(uint32_t received_id) {
	char i = 0;
	for (i; i < COM_PROTO_CALLBACKS_MAX_NUM; i++) {
		if (com_proto_callbacks[i].id == received_id) {
			com_proto_callbacks[i].callback();
			break;
		}
	}
}

void com_proto_process_rx_IRQ(uint32_t id, uint64_t data, uint8_t dlc) {
	xHigherPriorityTaskWoken = pdFALSE;
	if (com_proto_dst_from_id(id) == node_address) {
		if (!(rbuf32_put(&id_rbuf, (id & 0xffff))))
			com_proto_err_flag = 1;
		else {
			if (id & CMD_DATA_FLAG) {
				if (!(rbuf64_put(&data_rbuf, data)))
					com_proto_err_flag = 2;
			}
			xSemaphoreGiveFromISR(can_id_semphr, &xHigherPriorityTaskWoken);
			portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		}
	}
}

void com_proto_process_tx_IRQ(void) {
}

