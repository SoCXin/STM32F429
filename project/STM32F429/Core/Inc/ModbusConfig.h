
#ifndef THIRD_PARTY_MODBUS_LIB_CONFIG_MODBUSCONFIG_H_
#define THIRD_PARTY_MODBUS_LIB_CONFIG_MODBUSCONFIG_H_

//#define ENABLE_USB_CDC 1
#define ENABLE_TCP 1
#define ENABLE_USART_DMA 1

#define T35  5              // Timer T35 period (in ticks) for end frame detection.
#define MAX_BUFFER  128	    // Maximum size for the communication buffer in bytes.
#define TIMEOUT_MODBUS 1000 // Timeout for master query (in ticks)
#define MAX_M_HANDLERS 2    //Maximum number of modbus handlers that can work concurrently
#define MAX_TELEGRAMS 2     //Max number of Telegrams in master queue




#endif /* THIRD_PARTY_MODBUS_LIB_CONFIG_MODBUSCONFIG_H_ */
