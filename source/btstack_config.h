/*
 * btstack_config.h
 *
 * Created on: 19.01.2020
 *
 * @author Sven Fabricius
 * Contact: Sven.Fabricius@livediesel.de
 */

#ifndef BTSTACK_CONFIG_H_
#define BTSTACK_CONFIG_H_

// Port related features
#define HAVE_EMBEDDED_TIME_MS
#define HAVE_FREERTOS_TASK_NOTIFICATIONS

// BTstack features that can be enabled
#define ENABLE_BLE
//#define ENABLE_LOG_DEBUG
//#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_LE_PERIPHERAL
#define ENABLE_LE_CENTRAL

// BTstack configuration. buffers, sizes, ...
#define HCI_ACL_PAYLOAD_SIZE 200
#define MAX_NR_BNEP_SERVICES 0
#define MAX_NR_BNEP_CHANNELS 0
#define MAX_NR_HCI_CONNECTIONS 1
#define MAX_NR_L2CAP_SERVICES  0
#define MAX_NR_L2CAP_CHANNELS  0
#define MAX_NR_RFCOMM_MULTIPLEXERS 0
#define MAX_NR_RFCOMM_SERVICES 0
#define MAX_NR_RFCOMM_CHANNELS 0
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES  0
#define MAX_NR_GATT_CLIENTS 1
#define MAX_ATT_DB_SIZE 200
#define MAX_NR_HFP_CONNECTIONS 0
#define MAX_NR_WHITELIST_ENTRIES 1
#define MAX_NR_SM_LOOKUP_ENTRIES 3
#define MAX_NR_SERVICE_RECORD_ITEMS 1
#define MAX_NR_LE_DEVICE_DB_ENTRIES 1
#define NVM_NUM_DEVICE_DB_ENTRIES 4

// fixing unused in BTstack
#ifdef UNUSED
#undef UNUSED
#endif
#define UNUSED(variableName)	((void) variableName)

#endif /* BTSTACK_CONFIG_H_ */
