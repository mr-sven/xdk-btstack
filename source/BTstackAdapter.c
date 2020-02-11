/*
 * BTstackAdapter.c
 *
 * Created on: 20.01.2020
 *
 * @author Sven Fabricius
 * Contact: Sven.Fabricius@livediesel.de
 */

#include "BTstackAdapter.h"

/* system header files */
#include <stdio.h>

/* include FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* include XDK headers */
#include "XDK_Utils.h"
#include "BCDS_RingBuffer.h"
#include "BCDS_BSP_BT_EM9301.h"
#include "BCDS_MCU_UART.h"
#include "Serval_Clock.h"

/* include BTstack headers */
#include "btstack.h"
#include <platform/freertos/btstack_run_loop_freertos.h>
#include "btstack_chipset_em9301.h"

/**< Application controller task priority */
#define BTTASK_PRIO_APP_CONTROLLER                    (UINT32_C(2))

/**< Application controller task stack size */
#define BTTASK_STACK_SIZE_APP_CONTROLLER              (UINT32_C(1200))

/**< EM9301 uart handle */
static HWHandle_T em9301_uart_handle;

/**< rx buffer for uart handling */
static uint8_t mcu_uart_rx_buffer[1];

/**< own packet handler callback struct */
static btstack_packet_callback_registration_t hci_event_callback_registration;

/**< TaskHandle for the BTStack run loop */
static xTaskHandle btstack_task_handle = NULL;

/**< BTstack HAL CPU Implementation */
extern void hal_cpu_disable_irqs(void){}
extern void hal_cpu_enable_irqs(void){}
extern void hal_cpu_enable_irqs_and_sleep(void){}

/**< ISR Handler callbacks */
static void (*rx_done_handler)(void);
static void (*tx_done_handler)(void);

/**< rx ringbuffer size */
#define UART_EM9301_RING_BUFFER_SIZE  256

/**< rx ringbuffer */
RingBuffer_T transport_rx_ring_buffer;

/**< rx buffer for rx ringbuffer */
static uint8_t transport_rx_buffer[UART_EM9301_RING_BUFFER_SIZE];

/**< rx buffer pointer and size */
static uint8_t  * hal_uart_dma_rx_buffer;
static uint16_t   hal_uart_dma_rx_len;

/**< list of BTLE Versions */
static const char BT_VERS_NO[][4] = {
	"1.0",
	"1.1",
	"1.2",
	"2.0",
	"2.1",
	"3.0",
	"4.0",
	"4.1",
	"4.2",
	"5.0",
	"5.1",
	"5.2"
};

/**
 * @brief  BTstack HAL ms time implementation for wait processes
 */
extern uint32_t hal_time_ms(void)
{
	uint64_t time;
	Clock_getTimeMillis(&time);
	return (uint32_t) time;
}

/**
 * @brief  packet handler for some basic handling
 */
static void transport_packet_handler(uint8_t packet_type, uint16_t channel,	uint8_t *packet, uint16_t size)
{
	BCDS_UNUSED(channel);
	BCDS_UNUSED(size);
	if (packet_type != HCI_EVENT_PACKET)
		return;
	switch (hci_event_packet_get_type(packet))
	{
	case BTSTACK_EVENT_STATE:
		if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING)
		{
			bd_addr_t addr;
			gap_local_bd_addr(addr);
			printf("BTstack up and running at %s\r\n", bd_addr_to_str(addr));
		}
		break;
	case HCI_EVENT_COMMAND_COMPLETE:
		if (HCI_EVENT_IS_COMMAND_COMPLETE(packet, hci_read_local_version_information))
		{
        	uint16_t revision = little_endian_read_16(packet, 7);
        	uint16_t lmprevision = little_endian_read_16(packet, 12);
            printf("HCI Version: %s r%d\r\n", BT_VERS_NO[packet[6]], revision);
            printf("LMP Version: %s r%d\r\n", BT_VERS_NO[packet[9]], lmprevision);
		}
		break;
	case HCI_EVENT_COMMAND_STATUS:
		if (packet[2])
		{
			uint16_t opcode = little_endian_read_16(packet, 4);
			printf("Command status error 0x%02x %04x\r\n", packet[2], opcode);
		}
		break;
	case HCI_EVENT_VENDOR_SPECIFIC:
		switch (packet[2])
		{
		case HCI_SUBEVENT_EM_STANDBY_STATE:
			printf("Device in Standby State\r\n");
			break;
		default:
			break;
		}
		break;
	default:
    	log_debug("transport_packet_handler %02x: ", packet_type);
    	log_debug_hexdump(packet, size);
		break;
	}
}

/**
 * @brief   returns the current avilable size of BSP Ring Buffer
 */
int RingBuffer_BytesAvailable(RingBuffer_T * ringBuffer)
{
    int diff = ringBuffer->WriteIndex - ringBuffer->ReadIndex;
    if (diff >= 0) return diff;
    return diff + ringBuffer->Size;
}

/**
 * @brief   Executes the data transfer to bt stack
 */
static void hal_uart_em9301_transfer_rx_data(void)
{
    while (1)
    {
        if (!hal_uart_dma_rx_len) return;

        int bytes_available = RingBuffer_BytesAvailable(&transport_rx_ring_buffer);
        if (!bytes_available) return;

        int bytes_to_copy = btstack_min(bytes_available, hal_uart_dma_rx_len);
        uint32_t bytes_read = RingBuffer_Read(&transport_rx_ring_buffer, hal_uart_dma_rx_buffer, bytes_to_copy);
        hal_uart_dma_rx_buffer += bytes_read;
        hal_uart_dma_rx_len    -= bytes_read;

        if (hal_uart_dma_rx_len == 0)
        {
            (*rx_done_handler)();
            return;
        }
    }
}

/**
 * @brief   UART ISR event handler
 */
static void btstackAdapter_uart_event(UART_T uart, struct MCU_UART_Event_S event)
{
	BCDS_UNUSED(uart);
	if (event.RxComplete)
	{
		RingBuffer_Write(&transport_rx_ring_buffer, mcu_uart_rx_buffer, 1);
		// deliver new data
		hal_uart_em9301_transfer_rx_data();
	}
	else if (event.TxComplete)
	{
        (*tx_done_handler)();
	}
}

/**
 * @brief   close transport connection
 */
int hal_uart_dma_deinit(void)
{
	// disable device
	Retcode_T retcode = BSP_BT_EM9301_Disable();
	if (RETCODE_OK == retcode)
	{
		// deinit uart handle
		retcode = MCU_UART_Deinitialize(em9301_uart_handle);
	}
	if (RETCODE_OK == retcode)
	{
		// disconnect device
		retcode = BSP_BT_EM9301_Disconnect();
	}

	return 0;
}

/**
 * @brief   recevies a block of data
 */
void hal_uart_dma_receive_block(uint8_t *buffer, uint16_t length)
{
    hal_uart_dma_rx_buffer = buffer;
    hal_uart_dma_rx_len    = length;
    hal_uart_em9301_transfer_rx_data();
}

/**
 * @brief   sends a block of data
 */
void hal_uart_dma_send_block(const uint8_t *buffer, uint16_t length)
{
	// send packet
	MCU_UART_Send(em9301_uart_handle, (uint8_t *) buffer, length);
}

/**
 * @brief   sets the received ISR
 */
void hal_uart_dma_set_block_received( void (*the_block_handler)(void))
{
    rx_done_handler = the_block_handler;
}

/**
 * @brief   sets the sent ISR
 */
void hal_uart_dma_set_block_sent( void (*the_block_handler)(void))
{
    tx_done_handler = the_block_handler;
}

/**
 * @brief   sets baud rate
 */
int hal_uart_dma_set_baud(uint32_t baud)
{
	BCDS_UNUSED(baud);
    return 0;
}

/**
 * @brief   Initializes UART.
 */
void hal_uart_dma_init(void)
{
	// connect to EM9301
	Retcode_T retcode = BSP_BT_EM9301_Connect();
	if (RETCODE_OK == retcode)
	{
		// get EM9301 Uart Handle
		em9301_uart_handle = BSP_BT_EM9301_GetUARTHandle();

		// init UART and attach Event handler
		retcode = MCU_UART_Initialize(em9301_uart_handle, btstackAdapter_uart_event);
	}
	if (RETCODE_OK == retcode)
	{
		// enable EM9301
		retcode = BSP_BT_EM9301_Enable();
	}
	if (RETCODE_OK == retcode)
	{
		// start UART Receive
		retcode = MCU_UART_Receive(em9301_uart_handle, mcu_uart_rx_buffer, 1);
	}
	if (RETCODE_OK == retcode)
	{
		// reset device
		retcode = BSP_BT_EM9301_Reset();
	}
	if (RETCODE_OK == retcode)
	{
		// wakeup device
		retcode = BSP_BT_EM9301_SetWUHigh();
		vTaskDelay(1);
		retcode |= BSP_BT_EM9301_SetWULow();
	}
	if (RETCODE_OK != retcode)
	{
		printf("hal_uart_dma_init failed %u\r\n", retcode);
	}
}

// dummy config
static const hci_transport_config_uart_t config = {
    HCI_TRANSPORT_CONFIG_UART,
    115200,
    0,
    0,
    NULL
};

/**
 * @brief   Initializes the BTstack.
 */
static void btstack_init(void* pvParameters)
{
	BCDS_UNUSED(pvParameters);

	// setup memory
	btstack_memory_init();

	// init FreeRTOS instance
	btstack_run_loop_init(btstack_run_loop_freertos_get_instance());

	// init HCI
	hci_init(hci_transport_h4_instance(btstack_uart_block_freertos_instance()), &config);

	// load chipset
	hci_set_chipset(btstack_chipset_em9301_instance());

	// inform about BTstack state
	hci_event_callback_registration.callback = &transport_packet_handler;
	hci_add_event_handler(&hci_event_callback_registration);

    // init RX RingBuffer
	RingBuffer_Initialize(&transport_rx_ring_buffer, transport_rx_buffer, UART_EM9301_RING_BUFFER_SIZE);

	// execut run loop
	btstack_run_loop_execute();
}

/**
 * @brief   Initializes the BTstack Adapter module.
 * @details   Initializes the BTstack Adapter module, i.e. required resources (task, semaphores, queues, etc.) will be created and initialized.
 *
 * @note
 * 1. This API shall be called only once.
 *
 * @return  RETCODE_OK on success, or an error code otherwise. Refer #Retcode_General_E for other values.
 */
Retcode_T BTstackAdapter_Init(void)
{
	// create new task for bt stack adapter
	if (pdPASS != xTaskCreate(btstack_init, (const char * const ) "BTstackAdapter", BTTASK_STACK_SIZE_APP_CONTROLLER, NULL, BTTASK_PRIO_APP_CONTROLLER, &btstack_task_handle))
	{
		return RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
	}

	// wait for the task to start
	vTaskDelay(1000);

	return RETCODE_OK;
}

/**
 * @brief   Enabled the BTstack Adapter module.
 *
 * @note
 * 1. This API shall be called only once.
 *
 * @return  RETCODE_OK on success, or an error code otherwise. Refer #Retcode_General_E for other values.
 */
Retcode_T BTstackAdapter_Enable(void)
{
	// setup BT-MAC address
	uint8_t bleMacAddr[6] =	{ 0, 0, 0, 0, 0, 0 };
	Utils_GetMacInfoFromNVM(UTILS_BLE_MAC_DATA, bleMacAddr);
	hci_set_bd_addr(bleMacAddr);

	// turn on!
	hci_power_control(HCI_POWER_ON);

	btstack_run_loop_freertos_trigger();

	return RETCODE_OK;
}
