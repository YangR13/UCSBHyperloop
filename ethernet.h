#ifndef ETHERNET_H_
#define ETHERNET_H_

#include "board.h"

/* Wiznet Interrupt Input Pin */
#define WIZNET_INT_PORT					0
#define WIZNET_INT_PIN					4
#define WIZNET_IRQ_HANDLER     			GPIO_IRQHandler /* GPIO interrupt IRQ function name */
#define WIZNET_INTERRUPT_NVIC    		GPIO_IRQn   	/* GPIO interrupt NVIC interrupt name */

/* SSP Constants */
#define LPC_SSP           				LPC_SSP1
#define SSP_IRQ           				SSP1_IRQn
#define SSPIRQHANDLER     				SSP1_IRQHandler

/* Ethernet */
#define PROTO_UDP						0
#define PROTO_TCP						1

#define BUFFER_SIZE                    (0x0800)			// 2K
#define DATA_BUF_SIZE					BUFFER_SIZE - 4		// BUFFER_SIZE - (Header Size)
#define SSP_DATA_BITS                  (SSP_BITS_8)
#define SSP_DATA_BIT_NUM(databits)     (databits + 1)
#define SSP_DATA_BYTES(databits)       (((databits) > SSP_BITS_8) ? 2 : 1)
#define SSP_LO_BYTE_MSK(databits)      ((SSP_DATA_BYTES(databits) > 1) ? 0xFF : (0xFF >> \
												               (8 - SSP_DATA_BIT_NUM(databits))))
#define SSP_HI_BYTE_MSK(databits)      ((SSP_DATA_BYTES(databits) > 1) ? (0xFF >> \
												               (16 - SSP_DATA_BIT_NUM(databits))) : 0)
#define SSP_MODE_SEL                   (0x31)
#define SSP_TRANSFER_MODE_SEL          (0x32)
#define SSP_MASTER_MODE_SEL            (0x31)
#define SSP_SLAVE_MODE_SEL             (0x32)
#define SSP_POLLING_SEL                (0x31)
#define SSP_INTERRUPT_SEL              (0x32)
#define SSP_DMA_SEL                    (0x33)

/* Wiznet Registers 		*/
/* Initialization Registers */
#define MR 				0x0000	// Mode Register
#define GAR  			0x0001	// Gateway Address (4B)
#define SUBR 			0x0005	// Subnet Mask Address (4B)
#define SHAR 			0x0009	// Source Hardware Address (6B)
#define SIPR 			0x000F	// Source IP Address (4B)
#define IR				0x0015	// Interrupt Register
#define IMR 			0x0016 	// Socket Mask Register
#define RTR 			0x0017	// Retry Time-Value (2B)
#define RCR 			0x0019 	// Retry Count Register
#define IR2				0x0034	// Socket Interrupt Register
#define IMR2			0x0036	// General Interrupt Mask

#define RX_SIZE			  (0x800)	// 2K
#define TX_SIZE			  (0x800)	// 2K
#define RX_MAX_MASK		RX_SIZE-1
#define TX_MAX_MASK		TX_SIZE-1

/* Socket n (Sn) Control/Status Register Base Address */
#define Sn_MR_BASE 		0x4000	// Mode Register
#define Sn_CR_BASE		0x4001	// Command Register
#define Sn_IR_BASE		0x4002	// Interrupt Register
#define Sn_SR_BASE		0x4003	// Status Register
#define Sn_PORT_BASE	0x4004	// Source Port 			(2B)
#define Sn_DHAR_BASE	0x4006	// Destination MAC 		(6B)
#define Sn_DIPR_BASE	0x400C	// Destination IP 		(4B)
#define Sn_DPORT_BASE	0x4010	// Destination port 	(2B)
#define Sn_MSS_BASE		0x4012	// Maximum Segment Size (2B)
#define Sn_IMR_BASE		0x402C	// Socket Interrupt Mask

/* Socket Control Register Commands */
#define OPEN			0x01
#define LISTEN			0x02
#define CONNECT			0x04
#define DISCON			0x08
#define CLOSE			0x10
#define SEND			0x20
#define SEND_MAC		0x21
#define SEND_KEEP		0x22
#define RECV			0x40

/* Socket Interrupt Bitmasks */
#define SEND_OK			0x10
#define TIMEOUT			0x08
#define RECV_PKT		0x04
#define DISCON_SKT		0x02
#define Sn_CON			0x01

/* Socket n (Sn) Data Pointer Base Registers */
#define Sn_RXMEM_SIZE	0x401E	// Rx Memory size
#define Sn_TXMEM_SIZE	0x401F	// Tx Memory size
#define Sn_TX_FSR_BASE	0x4020	// Tx Free Size			(2B)
#define Sn_TX_RD_BASE	0x4022	// Tx Read Pointer		(2B)
#define Sn_TX_WR_BASE	0x4024	// Tx Write Pointer 	(2B)
#define Sn_RX_RSR_BASE	0x4026	// Rx Received Size 	(2B)
#define Sn_RX_RD_BASE	0x4028	// Rx Read Pointer 		(2B)
#define Sn_RX_WR_BASE	0x402A	// Rx Write Pointer 	(2B)

/* TCP/IP Defines */
#define REMOTE_IP0 		192
#define REMOTE_IP1 		168
#define REMOTE_IP2 		1
#define REMOTE_IP3 		100
#define REMOTE_PORT0 	0xA1 // 41234
#define REMOTE_PORT1 	0x12

/* Protocol methods */
#define AUT 			"AUT"
#define ACK 			"ACK"
#define BMP 			"BMP"
#define TMP 			"TMP"
#define POS 			"POS"
#define VEL 			"VEL"
#define ACC 			"ACC"
#define ROL 			"ROL"
#define PIT 			"PIT"
#define YAW 			"YAW"
#define PWR 			"PWR"
#define MSG 			"MSG"
#define PWR 			"PWR"
#define PASSWORD 		"gaucholoop"

#define SOCKET_ID 		0


enum Wiz_State {

	WIZ_IDLE,

	/* Message Send */
	START_SEND,
	TX_WP_READ,
	DATA_TRANSFER,
	TX_WP_WRITE,
	SEND_CMD,

	/* Message Receive */
	START_RECV,
	RX_RSR_READ,
	RX_ADDR_READ,
	DATA_RECV,
	RX_ADDR_WRITE,
	RECV_CMD

};

extern uint16_t gSn_RX_BASE[];
extern uint16_t gSn_TX_BASE[];

/* SPI Global Variables */
SSP_ConfigFormat ssp_format;
Chip_SSP_DATA_SETUP_T xf_setup;
uint8_t Tx_Buf[BUFFER_SIZE];
uint8_t Tx_Data[DATA_BUF_SIZE];
uint8_t Rx_Buf[BUFFER_SIZE];
uint8_t Rx_Data[DATA_BUF_SIZE];
uint8_t activeSockets, connectionOpen, connectionClosed, wiznetState;
uint16_t int_length;
uint16_t int_dst_mask, int_dst_ptr, int_wr_base, int_rd_ptr0, int_rd_ptr1;
uint16_t int_src_mask, int_src_ptr, int_rd_base;
volatile uint8_t isXferCompleted;
volatile uint8_t sendDataFlag;
volatile uint8_t wizIntFlag;

// DATA_BUF_SIZE is the size of a packet, which we don't expect to exceed
uint8_t Net_Tx_Data[DATA_BUF_SIZE];
uint8_t Net_Rx_Data[DATA_BUF_SIZE];

void sendData();
void recvData();
void SSPIRQHANDLER(void);
void WIZNET_IRQ_HANDLER(void);
void wizIntFunction();
void rec_method(char *method, char *val, int *val_len);
void send_method(char *method, char* val, int val_len);
void sendSensorDataTimerInit(LPC_TIMER_T * timer, uint8_t timerInterrupt, uint32_t tickRate);
void Wiz_Restart();
void Wiz_Init();
void Wiz_SSP_Init();
void Wiz_Network_Init();
void Wiz_Check_Network_Registers();
void Wiz_Memory_Init();
void Wiz_Int_Init(uint8_t n);
void Wiz_TCP_Init(uint8_t n);
void Wiz_UDP_Init(uint8_t n);
void Wiz_Destination_Init(uint8_t n);
void Wiz_Address_Check(uint8_t n);
void Wiz_TCP_Connect(uint8_t n);
void Wiz_TCP_Close(uint8_t n);
void Wiz_UDP_Close(uint8_t n);
void Wiz_Clear_Buffer(uint8_t n);
void ethernetInit(uint8_t protocol, uint8_t socket);
void Wiz_Deinit(uint8_t protocol, uint8_t socket);
void spi_Send_Int(uint16_t address, uint16_t length);
void spi_Recv_Int(uint16_t address, uint16_t length);
void spi_Send_Blocking(uint16_t address, uint16_t length);
void spi_Recv_Blocking(uint16_t address, uint16_t length);
void TIMER2_IRQHandler(void);
void sendSensorDataTimerInit(LPC_TIMER_T * timer, uint8_t timerInterrupt, uint32_t tickRate);
void Wiz_Xfer_Int(uint8_t n);

uint8_t Wiz_Check_Socket(uint8_t n);
uint8_t Wiz_Int_Clear(uint8_t n);
uint16_t Wiz_Send_Blocking(uint8_t n, uint8_t* message);
uint16_t Wiz_Recv_Blocking(uint8_t n, uint8_t* message);

#endif
