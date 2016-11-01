#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

//-----------------------------------------------
// Disables board-specific peripherals support
//-----------------------------------------------
//#define APP_DISABLE_BSP 1
#define APP_DISABLE_BSP 0

//-----------------------------------------------
// Includes board-specific peripherals support in application.
//-----------------------------------------------
#include <BoardConfig.h>

//-----------------------------------------------
// Atmel communication interfaces identifiers.
// Supported interfaces are platform and application dependent.
//-----------------------------------------------
#define APP_INTERFACE_USART 0x01
#define APP_INTERFACE_VCP 0x02
#define APP_INTERFACE_SPI 0x03
#define APP_INTERFACE_UART 0x04
#define APP_INTERFACE_USBFIFO 0x05

//-----------------------------------------------
// Atmel external memory identifiers.
// Supported memory is platform and application dependent.
//-----------------------------------------------
#define AT25F2048  0x01
#define AT45DB041  0x02
#define AT25DF041A 0x03

// Defines device role for boards witohut DIP switches. This parameter should not
// be confused with Library Type parameter. Device role must match library type if
// library type is not 'all'.
//#define APP_DEVICE_TYPE DEV_TYPE_COORDINATOR
//Device is End Device
#define APP_DEVICE_TYPE DEV_TYPE_ENDDEVICE
//Device is Router
//#define APP_DEVICE_TYPE DEV_TYPE_ROUTER

// Intervals in which coordinator and routers are sending state of their sensors.
#define APP_TIMER_SENDING_PERIOD 500

// Maximum amount of failed transmissions before network rejoin procedure is
// initiated.
#define APP_THRESHOLD_FAILED_TRANSMISSION 4

// Configuring device caption support. Caption will be shown in WSNMonitor tool
// window.
//#define APP_USE_DEVICE_CAPTION 1
#define APP_USE_DEVICE_CAPTION 0

// Support Atmel RCB KeyRemote board
#define APP_RCB_KEY_REMOTE_SUPPORT 0
//#define APP_RCB_KEY_REMOTE_SUPPORT 1

//-----------------------------------------------
//APP_USE_DEVICE_CAPTION == 1
//-----------------------------------------------
#if (APP_USE_DEVICE_CAPTION == 1)
  // Device caption, shown in WSNMonitor. Must be less, than 15 characters.
  #define APP_DEVICE_CAPTION "Ghost"
#endif

//-----------------------------------------------
//BOARD_SAM3S_EK
//-----------------------------------------------
#ifdef BOARD_SAM3S_EK
  // Defines primary serial interface type to be used by application.
  #define APP_INTERFACE APP_INTERFACE_USART
  //#define APP_INTERFACE APP_INTERFACE_UART
  //#define APP_INTERFACE APP_INTERFACE_VCP
  
  //-----------------------------------------------
  //APP_INTERFACE == APP_INTERFACE_UART
  //-----------------------------------------------
  #if (APP_INTERFACE == APP_INTERFACE_UART)
    // Defines UART interface name to be used by application.
    #define APP_UART_CHANNEL UART_CHANNEL_0
    //#define APP_UART_CHANNEL UART_CHANNEL_1
  #endif
  
  //-----------------------------------------------
  //APP_INTERFACE == APP_INTERFACE_VCP
  //-----------------------------------------------
  #if (APP_INTERFACE == APP_INTERFACE_VCP)
    // Defines USART interface name to be used by application.
    #define APP_USART_CHANNEL USART_CHANNEL_VCP
  #endif
  
  //-----------------------------------------------
  //APP_INTERFACE == APP_INTERFACE_USART
  //-----------------------------------------------
  #if (APP_INTERFACE == APP_INTERFACE_USART)
    // Defines USART interface name to be used by application.
    #define APP_USART_CHANNEL USART_CHANNEL_1
    //#define APP_USART_CHANNEL USART_CHANNEL_0
  #endif
  
  // Enable this option if target board suports controlling of TTL to RS232 converter
  // enable pin.
  #define BSP_ENABLE_RS232_CONTROL 1
  //#define BSP_ENABLE_RS232_CONTROL 0
#endif //

//-----------------------------------------------
//BOARD_RF231USBRD
//-----------------------------------------------
#ifdef BOARD_RF231USBRD
  // Defines primary serial interface type to be used by application.
  #define APP_INTERFACE APP_INTERFACE_VCP
  //#define APP_INTERFACE APP_INTERFACE_UART
  
  //-----------------------------------------------
  //APP_INTERFACE == APP_INTERFACE_UART
  //-----------------------------------------------
  #if (APP_INTERFACE == APP_INTERFACE_UART)
    // Defines UART interface name to be used by application.
    #define APP_UART_CHANNEL UART_CHANNEL_0
  #endif
  
  //-----------------------------------------------
  //APP_INTERFACE == APP_INTERFACE_VCP
  //-----------------------------------------------
  #if (APP_INTERFACE == APP_INTERFACE_VCP)
    // Defines USART interface name to be used by application.
    #define APP_USART_CHANNEL USART_CHANNEL_VCP
  #endif
#endif

//-----------------------------------------------
//BOARD_DERFUSB23E06
//-----------------------------------------------
#ifdef BOARD_DERFUSB23E06
  // Defines primary serial interface type to be used by application.
  #define APP_INTERFACE APP_INTERFACE_VCP
  //#define APP_INTERFACE APP_INTERFACE_UART

  //-----------------------------------------------
  //APP_INTERFACE == APP_INTERFACE_UART
  //-----------------------------------------------
  #if (APP_INTERFACE == APP_INTERFACE_UART)
    // Defines UART interface name to be used by application.
    #define APP_UART_CHANNEL UART_CHANNEL_0
  #endif

  //-----------------------------------------------
  //APP_INTERFACE == APP_INTERFACE_VCP
  //-----------------------------------------------
  #if (APP_INTERFACE == APP_INTERFACE_VCP)
    // Defines USART interface name to be used by application.
    #define APP_USART_CHANNEL USART_CHANNEL_VCP
  #endif
#endif

//-----------------------------------------------
//AT86RF212
//-----------------------------------------------
#ifdef AT86RF212
  // Enables or disables Listen Before Talk feature.
  #define CS_LBT_MODE false
  //#define CS_LBT_MODE true
  
  // 32-bit mask of channels to be scanned before network is started. Channels that
  // should be used are marked with logical 1 at corresponding bit location.
  //  Valid channel numbers for 2.4 GHz band are 0x0b - 0x1a
  //  Valid channel numbers for 900 MHz band are 0x00 - 0x0a
  // 
  //  Notes:
  //  1. for small amount of enabled channels it is more convinient to specify list
  // of channels in the form of '(1ul << 0x0b)'
  //  2. For 900 MHz band you also need to specify channel page
  // 
  //  Value range: 32-bit values:
  //  Valid channel numbers for 2.4 GHz band are 0x0b - 0x1a
  //  Valid channel numbers for 900 MHz band are 0x00 - 0x0a
  // 
  //  C-type: uint32_t
  //  Can be set: at any time before network start
  //  Persistent: Yes
  #define CS_CHANNEL_MASK (1L<<0x01)
  
  // Channel page number defines band and modulation scheme that will be used for
  // communication.
  // 
  //  Value range:
  //  0 - 915MHz (BPSK-40, channels 0x01 - 0x0a), 868MHz (BPSK-20, channel 0x00)
  //  2 - 915MHz (O-QPSK-250, channels 0x01 - 0x0a), 868Mhz (O-QPSK-100, channel
  // 0x00)
  //  5 - 780MHz (O-QPSK-250, channels 0x00 - 0x03, Chinese band)
  // 
  //  C-type: uint8_t
  //  Can be set: at any time before network start
  //  Persistent: Yes
  #define CS_CHANNEL_PAGE 0
  //O-QPSK
  //#define CS_CHANNEL_PAGE 2
  //Chinese band
  //#define CS_CHANNEL_PAGE 5
#else
  // 32-bit mask of channels to be scanned before network is started. Channels that
  // should be used are marked with logical 1 at corresponding bit location.
  //  Valid channel numbers for 2.4 GHz band are 0x0b - 0x1a
  //  Valid channel numbers for 900 MHz band are 0x00 - 0x0a
  // 
  //  Notes:
  //  1. for small amount of enabled channels it is more convinient to specify list
  // of channels in the form of '(1ul << 0x0b)'
  //  2. For 900 MHz band you also need to specify channel page
  // 
  //  Value range: 32-bit values:
  //  Valid channel numbers for 2.4 GHz band are 0x0b - 0x1a
  //  Valid channel numbers for 900 MHz band are 0x00 - 0x0a
  // 
  //  C-type: uint32_t
  //  Can be set: at any time before network start
  //  Persistent: Yes
  #define CS_CHANNEL_MASK (1L<<0x0e)
#endif

// The parameter specifies the predefined extended PANID of the network to be
// formed (for the coordinator) or joined (for a router or an end device). For a
// router or an end device the parameter can equal 0 allowing them to join the
// first suitable network that they discover.
// 
//  Value range: All 64-bit values except for 0xFFFFFFFFFFFFFFFFLL; specify a value
// in the 0x123456789ABCDEFLL format.
//  C-type: ExtPanId_t (equal to uint64_t)
//  Can be set: at any time before network start
//  Persistent: Yes
#define CS_EXT_PANID 0x0LL

// 64-bit Unique Identifier (UID) determining the device extended address. If this
// value is 0 stack will try to read hardware UID from external UID or EEPROM chip.
// at startup. Location of hardware UID is platform dependend and it may not be
// available on all platforms. If the latter case then UID value must be provided
// by user via this parameter. This parameter must be unique for each device in a
// network.
// 
//  Value range: any 64-bit value except for broadcast extended addresses
//  C-type: ExtAddr_t
//  Can be set: at any time before network start
//  Persistent: Yes
#define CS_UID 0x5ABE002LL

// Determines whether the static or automatic addressing mode will be used for the
// short address.
// 
//  If set to 1, the CS_NWK_ADDR parameter will be used as the device's short
// address. Otherwise, the short address is assigned automatically by the stack. An
// actual assignment method is specified in CS_ADDRESS_ASSIGNMENT_METHOD.
// 
//  Value range: 1 or 0
//  C-type: bool
//  Can be set: at any time before network start
//  Persistent: Yes
#define CS_NWK_UNIQUE_ADDR 0
//#define CS_NWK_UNIQUE_ADDR 1

//-----------------------------------------------
//CS_NWK_UNIQUE_ADDR == 1
//-----------------------------------------------
#if (CS_NWK_UNIQUE_ADDR == 1)
  // Specifies short (network) address if CS_NWK_UNIQUE_ADDR equals 1
  // 
  //  If static addressing is applied the stack uses the value of the parameter as a
  // short address. Otherwise, the stack assigns the parameter to a randomly chosen
  // value unique within the network. In both cases after the network start the
  // parameter holds actual short address of the device. While the device is in the
  // network its value must not be changed.
  // 
  //  Note: the coordinator short address always equals 0x0000.
  // 
  //  Value range: 0x0000 - 0xFFF8
  //  C-type: ShortAddr_t
  //  Can be set: at any time before network start
  //  Persistent: Yes
  #define CS_NWK_ADDR 0x0001
#endif

// The maximum number of direct children that a given device (the coordinator or a
// router) can have.
// 
//  The parameter is only enabled for routers and the coordinator. An end device
// can not have children. If an actual number of children reaches a parameter's
// value, the node will have not been able to accept any more children joining the
// network. The parameter can be set to 0 on a router thus preventing it from
// accepting any children and can help form a desired network topology. For
// example, if the parameter is set to 0 on all routers, then the coordinator will
// be the only device that can have children and the network will have star
// topology.
// 
//  Value range: from 0 to CS_NEIB_TABLE_SIZE
//  C-type: uint8_t
//  Can be set: at compile time only
//  Persistent: No
#define CS_MAX_CHILDREN_AMOUNT 8

// The maximum number of routers among the direct children of the device
// 
//  The parameter determines how many routers the device can have as children. Note
// that the maximum number of end devices is equal to CS_MAX_CHILDREN_AMOUNT -
// CS_MAX_CHILDREN_ROUTER_AMOUNT.
// 
//  Value range: from 0 to CS_MAX_CHILDREN_AMOUNT
//  C-type: uint8_t
//  Can be set: at compile time only
//  Persistent: No
#define CS_MAX_CHILDREN_ROUTER_AMOUNT 2

// End device sleep period given in milliseconds.
// 
//  On an end device this parameter determines the duration of a sleep period.
// Falling asleep is performed with the ZDO_SleepReq() request. After sleeping
// period exceeds the node is awakened and the application receives an indication
// via ZDO_WakeUpInd(). If the parameter's value is 0, then after the node falls
// asleep it can only be awakened by a hardware interrupt; a callback for a given
// IRQ is registered via HAL_RegisterIrq().
// 
//  On a router or the coordinator, the parameter is used in two ways:
// 
//  1) To remove information about lost child end devices. If a parent receives no
// data polls or data frames from the child end device for
// CS_NWK_END_DEVICE_MAX_FAILURES * (CS_END_DEVICE_SLEEP_PERIOD +
// CS_INDIRECT_POLL_RATE) ms, then it assumes it to be lost and deletes all
// information about such child.
// 
//  2) To determine whether to store or drop a message addressed to a child end
// device. The parent estimates the time when its child end device will wake up by
// adding this value to the moment when the last poll request has been received. If
// the time till end device wake up is greater than CS_MAC_TRANSACTION_TIME the
// frame is stored. Otherwise, the frame is dropped.
// 
//  Value range: any value valid for the C-type; add 'L' after a value
//  C-type: uint32_t
//  Can be set: at any time
//  Persistent: No
#define CS_END_DEVICE_SLEEP_PERIOD 2000L

//-----------------------------------------------
//STANDARD_SECURITY_MODE
//-----------------------------------------------
#ifdef STANDARD_SECURITY_MODE
  // The parameter is used to determine the security type.
  // 
  //  Value range: 0,3 - for standard security; 1,2 - for high security.
  //  0 - network key is preconfigured ;
  //  1 - network join without master key, but with a trust center link key, which
  // must be set via APS_SetLinkKey();
  //  2 - network join employs a master key, which must be set APS_SetMasterKey();
  //  3 - network key is no preconfigured, but rather received from the trust center
  // in an unencrypted frame. <br.
  //  C-type: uint8_t
  //  Can be set: at any time before network start
  //  Persistent: Yes
  #define CS_ZDO_SECURITY_STATUS 0
  //#define CS_ZDO_SECURITY_STATUS 3
  
  // Depending on security key type and security mode this is either network key,
  // master key, link key or initial link key.
  // 
  //  Network key is used to encrypt a part of a data frame occupied by the NWK
  // payload. This type of encryption is applied in both the standard and high
  // security mode. The high security mode also enables encryption of the APS payload
  // with a link key, but if the txOptions.useNwkKey field in APS request parameters
  // is set to 0, the APS payload is encrypted with the network key.
  // 
  //  The network key must be predefined if standard security is used with
  // CS_ZDO_SECURITY_STATUS set to 0. For all other values of CS_ZDO_SECURITY_STATUS
  // the network key is received from the trust center during device authentication.
  // Note that in the standard security mode with CS_ZDO_SECURITY_STATUS equal to 3
  // the network key is transferred to the joining device in an unencrypted frame.
  // 
  //  Value range: all 128-bit values
  //  C-type: uint8_t[16] or any compatible
  //  Can be set: at any time before network start
  //  Persistent: No
  #define CS_NETWORK_KEY {0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC}
  
  // The maximum number of network keys that can be stored on the device
  // 
  //  A device in a secured network can keep several network keys up to the value of
  // this parameter. Upon frame reception the device extracts key sequence number
  // from the auxiliary header of the frame and decrypts the message with the network
  // key corresponding to this sequence number. Besides, one key is considered active
  // for each device; this is the key that is used for encrypting outgoing frames.
  // The keys are distributed by the trust center with the help of the
  // APS_TransportKeyReq() command. The trust center can also change the active key
  // of a remote node via a call to APS_SwitchKeyReq().
  // 
  //  C-type: NwkKeyAmount_t (typedef for uint8_t)
  //  Can be set: at compile time only
  //  Persistent: No
  #define CS_NWK_SECURITY_KEYS_AMOUNT 1
  
  // Address of device responsible for authentication and key distribution (Trust
  // Center).
  // 
  //  The parameter specifies the trust center extended address. The stack makes use
  // of the parameter to support various operations in networks with security
  // enabled. For correct network operation a parameter's value must coincide with
  // the actual trust center address.
  // 
  //  In case the trust center extended address is unknown, for example, for testing
  // purposes, the parameter can be assigned to the universal trust center address
  // which equals APS_SM_UNIVERSAL_TRUST_CENTER_EXT_ADDRESS.
  // 
  //  Value range: All 64-bit values except for 0xFFFFFFFFFFFFFFFFLL, specify a value
  // in the 0x123456789ABCDEFLL format.
  //  C-type: ExtAddr_t
  //  Can be set: at any time before network start
  //  Persistent: Yes
  #define CS_APS_TRUST_CENTER_ADDRESS 0xAAAAAAAAAAAAAAAALL
  
  // Security information waiting timeout before secure network join considered
  // failed.
  // 
  //  A timeout is started when connection with a parent is established. If the
  // security related procedures that are performed after this will not be completed
  // before the timeout exceeds, the device will fail joining the network. A value is
  // measured in milliseconds.
  // 
  //  C-type: uint32_t
  //  Can be set: at compile time only
  //  Persistent: No
  #define CS_APS_SECURITY_TIMEOUT_PERIOD 10000
#endif

//-----------------------------------------------
//STDLINK_SECURITY_MODE
//-----------------------------------------------
#ifdef STDLINK_SECURITY_MODE
  // The parameter enabled in the high security mode specifies the size of the APS
  // key-pair set. The APS key-pair set stores pairs of corresponding extended
  // address and a link key or a master key. For each node with which the current
  // node is going to communicate it must keep an entry with the remote node extended
  // address and a link key. If the link key is unknown, the node can request the
  // trust center for it via APS_RequestKeyReq(). The trust center must store a link
  // key or a master key depending on the CS_SECURITY_STATUS used for each node it is
  // going to authenticate. Entries can also be added manually by APS_SetLinkKey()
  // and APS_SetMasterKey().
  // 
  //  Value range: 1 - 255
  //  C-type: uint8_t
  //  Can be set: at compile time only
  //  Persistent: No
  #define CS_APS_KEY_PAIR_DESCRIPTORS_AMOUNT 8
  
  // Network address of device responsible for authentication and key distribution
  // (Trust Center).
  // 
  //  The parameter specifies the trust center short address. The stack makes use of
  // the parameter to support various operations in networks with security enabled.
  // For correct network operation the parameter's value must coincide with the
  // actual trust center address.
  // 
  //  Value range: All 16-bit values except for broadcast addresses, specify a value
  // in the hex format.
  //  C-type: ShortAddr_t
  //  Can be set: at any time before network start
  //  Persistent: Yes
  #define CS_SHORT_TRUST_CENTER_ADDRESS 0x0000
  
  // Address of device responsible for authentication and key distribution (Trust
  // Center).
  // 
  //  The parameter specifies the trust center extended address. The stack makes use
  // of the parameter to support various operations in networks with security
  // enabled. For correct network operation a parameter's value must coincide with
  // the actual trust center address.
  // 
  //  In case the trust center extended address is unknown, for example, for testing
  // purposes, the parameter can be assigned to the universal trust center address
  // which equals APS_SM_UNIVERSAL_TRUST_CENTER_EXT_ADDRESS.
  // 
  //  Value range: All 64-bit values except for 0xFFFFFFFFFFFFFFFFLL, specify a value
  // in the 0x123456789ABCDEFLL format.
  //  C-type: ExtAddr_t
  //  Can be set: at any time before network start
  //  Persistent: Yes
  #define CS_APS_TRUST_CENTER_ADDRESS 0xAAAAAAAAAAAAAAAALL
  
  // Security information waiting timeout before secure network join considered
  // failed.
  // 
  //  A timeout is started when connection with a parent is established. If the
  // security related procedures that are performed after this will not be completed
  // before the timeout exceeds, the device will fail joining the network. A value is
  // measured in milliseconds.
  // 
  //  C-type: uint32_t
  //  Can be set: at compile time only
  //  Persistent: No
  #define CS_APS_SECURITY_TIMEOUT_PERIOD 10000
  
  // Depending on security key type and security mode this is either network key,
  // master key, link key or initial link key.
  // 
  //  Network key is used to encrypt a part of a data frame occupied by the NWK
  // payload. This type of encryption is applied in both the standard and high
  // security mode. The high security mode also enables encryption of the APS payload
  // with a link key, but if the txOptions.useNwkKey field in APS request parameters
  // is set to 0, the APS payload is encrypted with the network key.
  // 
  //  The network key must be predefined if standard security is used with
  // CS_ZDO_SECURITY_STATUS set to 0. For all other values of CS_ZDO_SECURITY_STATUS
  // the network key is received from the trust center during device authentication.
  // Note that in the standard security mode with CS_ZDO_SECURITY_STATUS equal to 3
  // the network key is transferred to the joining device in an unencrypted frame.
  // 
  //  Value range: all 128-bit values
  //  C-type: uint8_t[16] or any compatible
  //  Can be set: at any time before network start
  //  Persistent: No
  #define CS_NETWORK_KEY {0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC}
  
  // The parameter is used to determine the security type.
  // 
  //  Value range: 0,3 - for standard security; 1,2 - for high security.
  //  0 - network key is preconfigured ;
  //  1 - network join without master key, but with a trust center link key, which
  // must be set via APS_SetLinkKey();
  //  2 - network join employs a master key, which must be set APS_SetMasterKey();
  //  3 - network key is no preconfigured, but rather received from the trust center
  // in an unencrypted frame. <br.
  //  C-type: uint8_t
  //  Can be set: at any time before network start
  //  Persistent: Yes
  #define CS_ZDO_SECURITY_STATUS 1
  //#define CS_ZDO_SECURITY_STATUS 2
  
  // The maximum number of network keys that can be stored on the device
  // 
  //  A device in a secured network can keep several network keys up to the value of
  // this parameter. Upon frame reception the device extracts key sequence number
  // from the auxiliary header of the frame and decrypts the message with the network
  // key corresponding to this sequence number. Besides, one key is considered active
  // for each device; this is the key that is used for encrypting outgoing frames.
  // The keys are distributed by the trust center with the help of the
  // APS_TransportKeyReq() command. The trust center can also change the active key
  // of a remote node via a call to APS_SwitchKeyReq().
  // 
  //  C-type: NwkKeyAmount_t (typedef for uint8_t)
  //  Can be set: at compile time only
  //  Persistent: No
  #define CS_NWK_SECURITY_KEYS_AMOUNT 1
#endif //

// Maximum amount of records in the Neighbor Table.
// 
//  The parameter determines the size of the neighbor table which is used to store
// beacon responses from nearby devices. The parameter puts an upper bound over the
// amount of child devices possible for the node.
// 
//  Value range: at minimum 1, the maximum value is limited to the available memory
//  C-type: uint8_t
//  Can be set: at compile time only
//  Persistent: No
#define CS_NEIB_TABLE_SIZE 8

// Maximum amount of records in the network Route Table.
// 
//  The parameter sets the maximum number of records that can be kept in the NWK
// route table. The table is used by NWK to store information about established
// routes. Each table entry specifies the next-hop short address for a route from
// the current node to a given destination node. The table is being filled
// automatically during route discovery. An entry is added when a route is
// discovered.
// 
//  Since the end device always sends a frame directly to its parent its route
// table size should be set to 0.
// 
//  C-type: uint8_t
//  Can be set: at compile time only
//  Persistent: No
#define CS_ROUTE_TABLE_SIZE 10

// Maximum amount of records in the network Address Map Table.
// 
//  The parameter sets the maximum number of records in the address map table used
// by NWK to store pairs of corresponding short and extended addresses. The stack
// appeals to the table when a data frame is being sent to a specified extended
// address to extract the corresponding short address. If it fails to find the
// short address, an error is reported.
// 
//  C-type: NwkSizeOfAddressMap_t (typedef for uint8_t)
//  Can be set: at compile time only
//  Persistent: No
#define CS_ADDRESS_MAP_TABLE_SIZE 5

// Maximum amount of records in the network Route Discovery Table.
// 
//  The parameter specifies the size of the route discovery table used by NWK to
// store next-hop addresses of the nodes for routes that are not yet established.
// Upon exhausting the capacity of the table, the stack starts rewriting old
// entries. If the size of the route table is big enough after all used routes are
// established the table may not be used.
// 
//  Since the end device always sends a frame directly to its parent its route
// discovery table size should be set to 0.
// 
//  C-type: uint8_t
//  Can be set: at compile time only
//  Persistent: No
#define CS_ROUTE_DISCOVERY_TABLE_SIZE 3

// Maximum amount of records in the Duplicate Rejection Table.
// 
//  The duplicate rejection table is used by APS to store information about
// incoming unicast messages in order to reject messages that have been already
// received and processed. Following ZigBee specification, the parameter should be
// not less than 1.
// 
//  Value range: greater than 1
//  C-type: uint8_t
//  Can be set: at compile time only
//  Persistent: No
#define CS_DUPLICATE_REJECTION_TABLE_SIZE 8

// Maximum amount of records in the Broadcast Transaction Table.
// 
//  The broadcast transmission table is used for tracking incoming broadcast
// messages to mark messages that have already been processed by the node. This
// causes only one copy for each broadcast message to be processed. An entry for a
// broadcast message is stored for a certain period of time and then removed.
// 
//  C-type: uint8_t
//  Can be set: at compile time only
//  Persistent: No
#define CS_NWK_BTT_SIZE 8

// The number of buffers for data requests on the APS layer.
// 
//  The parameter specifies the number of buffers that are allocated by APS to
// store data requests parameters. The parameter puts an upper bound to the number
// of data requests that can be processed by APS simultaneously. If all buffers are
// in use and a new data request appears, it is kept in a queue until a buffer is
// released.
// 
//  C-type: uint8_t
//  Can be set: at compile time only
//  Persistent: No
#define CS_APS_DATA_REQ_BUFFERS_AMOUNT 3

// The number of buffers for acknowledgement messages sent by APS.
// 
//  This parameter determines the amount of memory that needs to be allocated for a
// special type of buffers used by APS to store payloads for acknowledgement
// frames. The need to use the buffers occurs when the node receives a frame that
// has to be acknowledged. That is, the APS component on the node has to send an
// acknowledgement frame. For frames initiated by the application, the memory for a
// payload is to be allocated by the application on its own, while the payload
// memory for an acknowledgement frame shall be reserved by APS. The request
// parameters are still stored in the data request buffers.
// 
//  Typically, a value of this parameter equals CS_APS_DATA_REQ_BUFFERS_AMOUNT - 1.
// 
//  C-type: uint8_t
//  Can be set: at compile time only
//  Persistent: No
#define CS_APS_ACK_FRAME_BUFFERS_AMOUNT 2

// Amount of buffers to keep MAC data indications on network layer.
#define CS_NWK_BUFFERS_AMOUNT 4

// The parameter specifies the TX power of the transceiver device, is measured in
// dBm(s). After the node has entered the network the value can only be changed via
// the ZDO_SetTxPowerReq() function.
// 
//  Value range: depends on the hardware. Transmit power must be in the range from
// -17 to 3 dBm for AT86RF231, AT86RF230 and AT86RF230B. For AT86RF233 transmit
// power must be in the range from -17 to 4 dBm. For AT86RF212 transmit power must
// be in the range from -11 to 11 dBm.
// 
//  C-type: int8_t
//  Can be set: at any time
//  Persistent: Yes
#define CS_RF_TX_POWER 3


#endif // _CONFIGURATION_H_
