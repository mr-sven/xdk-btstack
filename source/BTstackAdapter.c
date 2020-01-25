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

/**< size of the rx ringbuffer */
#define TRANSPORT_READ_BUFFER_SIZE	0x100

/**< rx ringbuffer */
RingBuffer_T transport_rx_ring_buffer;

/**< rx buffer for rx ringbuffer */
static uint8_t transport_rx_buffer[TRANSPORT_READ_BUFFER_SIZE];

/**< transport tx semaphore mutex */
static SemaphoreHandle_t transport_tx_signal;

/**< EM9301 uart handle */
static HWHandle_T em9301_uart_handle;

/**< rx buffer for uart handling */
static uint8_t mcu_uart_rx_buffer[1];

/**< own packet handler callback struct */
static btstack_packet_callback_registration_t hci_event_callback_registration;

/**< data source for integration with BTstack Runloop */
static btstack_data_source_t transport_data_source;

/**< packet send signal for BTstack */
static int transport_signal_sent;

/**< packet received signal for BTstack */
static volatile int transport_packets_to_deliver = 0;

/**< BTStack callback for packet handling */
static void (*btstack_packet_handler)(uint8_t packet_type, uint8_t *packet, uint16_t size);

/**< TaskHandle for the BTStack run loop */
static xTaskHandle btstack_task_handle = NULL;

// BTstack HAL CPU Implementation
extern void hal_cpu_disable_irqs(void){ }
extern void hal_cpu_enable_irqs(void) { }
extern void hal_cpu_enable_irqs_and_sleep(void) { }

/**
 * @brief  BTstack HAL ms time implementation for wait processes
 */
extern uint32_t hal_time_ms(void)
{
	uint64_t time;
	Clock_getTimeMillis(&time);
    return (uint32_t)time;
}


/**
 * @brief  packet handler for some basic handling
 */
static void transport_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
	BCDS_UNUSED(channel);
	BCDS_UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;
    switch(hci_event_packet_get_type(packet))
    {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING)
            {
                bd_addr_t addr;
                gap_local_bd_addr(addr);
                printf("BTstack up and running at %s\r\n",  bd_addr_to_str(addr));
            }
            break;
        case HCI_EVENT_COMMAND_COMPLETE:
            if (HCI_EVENT_IS_COMMAND_COMPLETE(packet, hci_read_local_version_information))
            {
            	uint16_t revision = little_endian_read_16(packet, 5);
            	uint16_t lmprevision = little_endian_read_16(packet, 10);
                printf("HCI Version: 1.0\r\n");
                printf("HCI Revision: %d\r\n", revision);
                printf("LMP Version: %d\r\n", packet[7]);
                printf("LMP Revision: %d\r\n", lmprevision);
            }
            break;
        case HCI_EVENT_COMMAND_STATUS:
        	if (packet[2])
        	{
        		uint16_t opcode = little_endian_read_16(packet, 4);
        		printf("Command status error 0x%02x %04x\r\n", packet[2], opcode);
        	}
        	break;
        default:
            break;
    }
}

/**< Maximum data to receive = 1 byte packet type, 2 byte command, 1 byte length, max 265 byte data */
#define MAX_RX_DATA_SIZE		(1 + 2 + 1 + 0xff)

/**< data to read counter for rx buffer */
static uint16_t rx_data_to_recv = MAX_RX_DATA_SIZE;

/**
 * @brief   UART ISR event handler
 */
static void btstackAdapter_uart_event(UART_T uart, struct MCU_UART_Event_S event)
{
    BCDS_UNUSED(uart);
	if (event.RxComplete)
	{
		// store data to ring buffer
		RingBuffer_Write(&transport_rx_ring_buffer, mcu_uart_rx_buffer, 1);

		// position of packet Type
		if (rx_data_to_recv == MAX_RX_DATA_SIZE && mcu_uart_rx_buffer[0] == HCI_EVENT_PACKET)
		{
			// event packet is one byte shorter because event-type is only one byte long
			rx_data_to_recv--;
		}

		// count to zero
		rx_data_to_recv--;

		// position of content length
		if (rx_data_to_recv == 0xff)
		{
			rx_data_to_recv = mcu_uart_rx_buffer[0];
		}

		// all data read
		if (rx_data_to_recv == 0)
		{
			// reset data to read
			rx_data_to_recv = MAX_RX_DATA_SIZE;

			// increment current available packet counter
			transport_packets_to_deliver++;

			// inform BTstack loop
			btstack_run_loop_freertos_trigger_from_isr();
		}
	}
	else if(event.TxComplete)
	{
		// inform TX mutex
	    BaseType_t higherPriorityTaskWoken = pdFALSE;
        if (xSemaphoreGiveFromISR(transport_tx_signal, &higherPriorityTaskWoken) == pdTRUE)
        {
            portYIELD_FROM_ISR(higherPriorityTaskWoken);
        }
	}
}

/**
 * @brief   notify BTstack that packed was send
 */
static void transport_notify_packet_send(void)
{
    // notify upper stack that it might be possible to send again
    uint8_t event[] = { HCI_EVENT_TRANSPORT_PACKET_SENT, 0};
    btstack_packet_handler(HCI_EVENT_PACKET, &event[0], sizeof(event));
}

/**< receive buffer for packet handler */
static uint8_t hci_receive_buffer[MAX_RX_DATA_SIZE];

/**< receive buffer packet type */
static uint8_t hci_receive_packet_type;

/**< receive buffer packet length */
static uint8_t hci_receive_packet_len;

/**
 * @brief   deliver packets to BTstack
 */
static void transport_deliver_packets(void)
{
    while (transport_packets_to_deliver > 0)
    {
    	transport_packets_to_deliver--;

    	// read packet type
        RingBuffer_Read(&transport_rx_ring_buffer, &hci_receive_packet_type, 1);

        if (hci_receive_packet_type == HCI_COMMAND_DATA_PACKET)
        {
        	// read command (2 byte) and length (1 byte)
            RingBuffer_Read(&transport_rx_ring_buffer, hci_receive_buffer, 3);

        	// length from index 2
            hci_receive_packet_len = hci_receive_buffer[2];

            // read data to buffer with length
            RingBuffer_Read(&transport_rx_ring_buffer, &hci_receive_buffer[3], hci_receive_packet_len);

            // packet length plus command and length field
            hci_receive_packet_len += 3;
        }
        else if (hci_receive_packet_type == HCI_EVENT_PACKET)
        {
        	// read event (1 byte) and length (1 byte)
            RingBuffer_Read(&transport_rx_ring_buffer, hci_receive_buffer, 2);

        	// length from index 1
            hci_receive_packet_len = hci_receive_buffer[1];

            // read data to buffer with length
            RingBuffer_Read(&transport_rx_ring_buffer, &hci_receive_buffer[2], hci_receive_packet_len);

            // packet length plus event and length field
            hci_receive_packet_len += 2;
        }

        // handle package
        btstack_packet_handler(hci_receive_packet_type, hci_receive_buffer, hci_receive_packet_len);
    }
}

/**
 * @brief   transport process callback
 */
static void transport_process(btstack_data_source_t *ds, btstack_data_source_callback_type_t callback_type)
{
    BCDS_UNUSED(ds);
    switch (callback_type)
    {
        case DATA_SOURCE_CALLBACK_POLL:
            if (transport_signal_sent)
            {
                transport_signal_sent = 0;
                transport_notify_packet_send();
            }
            if (transport_packets_to_deliver)
            {
                transport_deliver_packets();
            }
            break;
        default:
            break;
    }
}

/**
 * @brief   init transport
 * @param transport_config
 */
static void transport_init(const void *transport_config)
{
    BCDS_UNUSED(transport_config);

    // init RX RingBuffer
	RingBuffer_Initialize(&transport_rx_ring_buffer, transport_rx_buffer, TRANSPORT_READ_BUFFER_SIZE);

	// create TX Semaphore Mutex
	transport_tx_signal = xSemaphoreCreateBinary();

    // set up polling data_source
    btstack_run_loop_set_data_source_handler(&transport_data_source, &transport_process);
    btstack_run_loop_enable_data_source_callbacks(&transport_data_source, DATA_SOURCE_CALLBACK_POLL);
    btstack_run_loop_add_data_source(&transport_data_source);
}

/**
 * @brief   open transport connection
 */
static int transport_open(void)
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
        printf("transport_open failed %u\r\n", retcode);
        return -1;
    }

    return 0;
}

/**
 * @brief   close transport connection
 */
static int transport_close(void)
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
 * @brief   packets can always be send due Semaphore mutex lock
 */
static int transport_can_send_packet_now(uint8_t packet_type)
{
    BCDS_UNUSED(packet_type);
    return 1;
}

/**
 * @brief   send packet to chipset
 */
static int transport_send_packet(uint8_t packet_type, uint8_t *packet, int size)
{
    // store packet type before actual data and increase size
    size++;
    packet--;
    *packet = packet_type;

    // send packet
	xSemaphoreTake(transport_tx_signal, 0);
	Retcode_T retcode = MCU_UART_Send(em9301_uart_handle, packet, size);
	if (RETCODE_OK == retcode)
	{
		if (pdTRUE != xSemaphoreTake(transport_tx_signal, (TickType_t) 1000))
		{
			retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_TIMEOUT);
		}
	}
	if (RETCODE_OK == retcode)
	{
		transport_signal_sent = 1;
		btstack_run_loop_freertos_trigger();
	}
    return 0;
}

/**
 * @brief   register packet handler for HCI packets: ACL, SCO, and Events
 */
static void transport_register_packet_handler(void (*handler)(uint8_t packet_type, uint8_t *packet, uint16_t size))
{
    btstack_packet_handler = handler;
}

/**< BTstack transport callbacks */
static const hci_transport_t transport = {
    "xdk-vhci",
    &transport_init,
    &transport_open,
    &transport_close,
    &transport_register_packet_handler,
    &transport_can_send_packet_now,
    &transport_send_packet,
    NULL, // set baud rate
    NULL, // reset link
    NULL, // set SCO config
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
    hci_init(&transport, NULL);

    // load chipset
	hci_set_chipset(btstack_chipset_em9301_instance());

    // inform about BTstack state
    hci_event_callback_registration.callback = &transport_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

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
    uint8_t bleMacAddr[6] = {0, 0, 0, 0, 0, 0};
	Utils_GetMacInfoFromNVM(UTILS_BLE_MAC_DATA, bleMacAddr);
    hci_set_bd_addr(bleMacAddr);

    // turn on!
    hci_power_control(HCI_POWER_ON);

    btstack_run_loop_freertos_trigger();

	return RETCODE_OK;
}
