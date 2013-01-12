/************************************************************************
 * NormalException.net Software, and other contributors
 * http://www.normalexception.net
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author: Paimon Sorornejad
 * Version: v1.12
 * Revision Date: 1/11/2013
 *
 * Reference: http://ww1.microchip.com/downloads/en/devicedoc/21801d.pdf
 ************************************************************************/

//Filter Buffer Constants
#define BUFFER_0     0
#define BUFFER_1     1
#define BUFFER_ALL   255

#define READ_RX_0_SIDH 0x00
#define READ_RX_0_DATA 0x02
#define READ_RX_1_SIDH 0x04
#define READ_RX_1_DATA 0x06

// Acceptance Filter 0
#define RXF0SIDH 0x00	// Standard ID High
#define RXF0SIDL 0x01	// Standard ID Low
#define RXF0EID8 0x02	// Extended ID High
#define RXF0EID0 0x03	// Extended ID Low

// Acceptance Filter 1
#define RXF1SIDH 0x04	// Standard ID High
#define RXF1SIDL 0x05	// Standard ID Low
#define RXF1EID8 0x06	// Extended ID High
#define RXF1EID0 0x07	// Extended ID Low

// Acceptance Filter 2
#define RXF2SIDH 0x08	// Standard ID High
#define RXF2SIDL 0x09	// Standard ID Low
#define RXF2EID8 0x0A	// Extended ID High
#define RXF2EID0 0x0B	// Extended ID Low

// Acceptance Filter 3
#define RXF3SIDH 0x10	// Standard ID High
#define RXF3SIDL 0x11	// Standard ID Low
#define RXF3EID8 0x12	// Extended ID High
#define RXF3EID0 0x13	// Extended ID Low

// Acceptance Filter 4
#define RXF4SIDH 0x14	// Standard ID High
#define RXF4SIDL 0x15	// Standard ID Low
#define RXF4EID8 0x16	// Extended ID High
#define RXF4EID0 0x17	// Extended ID Low

// Acceptance Filter 5
#define RXF5SIDH 0x18	// Standard ID High
#define RXF5SIDL 0x19	// Standard ID Low
#define RXF5EID8 0x1A	// Extended ID High
#define RXF5EID0 0x1B	// Extended ID Low

#define BFPCTRL 0x0C	// Buffer Pin Control
#define TXRTSCTRL 0x0D	// TX Pin Control

#define CANSTAT 0x0E	// CAN Status Register
#define CANCTRL 0x0F	// CAN Control Register

#define TEC 0x1C		// Transmission Error Counter
#define REC 0x1D		// Recieve Error Counter

#define RXM0SIDH 0x20	// RX Mask Standard ID High
#define RXM0SIDL 0x21	// RX Mask Standard ID Low
#define RXM0EID8 0x22	// RX Mask Extended ID High
#define RXM0EID0 0x23	// RX Mask Extended ID Low

#define RXM1SIDH 0x24	// RX Mask Standard ID High
#define RXM1SIDL 0x25	// RX Mask Standard ID Low
#define RXM1EID8 0x26	// RX Mask Extended ID High
#define RXM1EID0 0x27	// RX Mask Extended ID Low

#define CNF3 0x28		// Configuration Register 3
	#define WAKFIL  6
	#define PHSEG22	2
	#define PHSEG21	1
	#define PHSEG20	0
#define CNF2 0x29       // Configuration Register 2
#define CNF1 0x2A       // Configuration Register 1
	#define SJW1 7
	#define SJW0 6
	#define BRP5 5
	#define BRP4 4
	#define BRP3 3
	#define BRP2 2
	#define BRP1 1
	#define BRP0 0

#define CANINTE 0x2B	// CAN Interrupt Enable
	#define MERRE 7			// Message Error Interrupt Enable Bit
	#define WAKIE 6			// Wakeup Interrupt Enable Bit
	#define ERRIE 5			// Error Interrupt Enable Bit
	#define TX2IE 4			// Transmit Buffer 2 Empty Enable Interrupt Bit
	#define TX1IE 3			// Transmit Buffer 1 Empty Enable Interrupt Bit
	#define TX0IE 2			// Transmit Buffer 0 Empty Enable Interrupt Bit
	#define RX1IE 1			// Receive Buffer 1 Empty Enable Interrupt Bit
	#define RX0IE 0			// Receive Buffer 0 Empty Enable Interrupt Bit
#define CANINTF 0x2C	// CAN Interrupt Flag
	#define MERRF 7			// Message Error Interrupt Flag
	#define WAKIF 6			// Wakeup Interrupt Flag
	#define ERRIF 5			// Error Interrupt Flag
	#define TX2IF 4			// Transmit Buffer 2 Interrupt Flag
	#define TX1IF 3			// Transmit Buffer 1 Interrupt Flag
	#define TX0IF 2			// Transmit Buffer 0 Interrupt Flag
	#define RX1IF 1			// Receive Buffer 1 Interrupt Flag
	#define RX0IF 0			// Receive Buffer 0 Interrupt Flag
	
#define EFLG 0x2D		// Error Flag
#define TXB0CTRL 0x30	// Transmit Buffer 0 Control Register
	#define TXREQ 3
#define TXB0SIDH 0x31	// Transmit Buffer 0 Standard ID High
#define TXB0SIDL 0x32	// Transmit Buffer 0 Standard ID Low
	#define EXIDE 3
#define TXB0EID8 0x33	// Transmit Buffer 0 Extended ID High
#define TXB0EID0 0x34	// Transmit Buffer 0 Extended ID High
#define TXB0DLC 0x35	// Transmit Buffer Data Length
  #define TXRTR 7		
#define TXB0D0 0x36		// Transmit Buffer 0 Data Byte 0

// Buffer 0 Control
#define RXB0CTRL 0x60	// Receive Buffer 0 Control Register
	#define RXM1 6			// Receive Buffer Operating Mode
	#define RXM0 5			// Receive Buffer Operating Mode
	#define RXRTR 3			// Receive Remote Transfer Request bit
	// Bits 2:0 FILHIT2:0
#define RXB0SIDH 0x61	// Receive Buffer 0 Standard ID High
#define RXB0SIDL 0x62	// Receive Buffer 0 Standard ID Low
#define RXB0EID8 0x63	// Receive Buffer 0 Extended ID High
#define RXB0EID0 0x64	// Receive Buffer 0 Extended ID Low
#define RXB0DLC 0x65	// Receive Buffer 0 Data Length Code
  #define RTR  6
  #define DLC3 3
  #define DLC2 2
  #define DLC1 1
  #define DLC0 0
#define RXB0D0 0x66		// Receive Buffer 0 Data Byte 0

// Buffer 1 Control
#define RXB1CTRL	0x70 // Receive Buffer 1 Control Register
#define RXB1SIDH	0x71 // Receive Buffer 1 Standard ID High
#define RXB1SIDL	0x72 // Receive Buffer 1 Standard ID Low
#define RXB1EID8	0x73 // Receive Buffer 1 Extended ID High
#define RXB1EID0	0x74 // Receive Buffer 1 Extended ID Low
#define RXB1DLC	    0x75 // Receive Buffer 1 Data Length Code
#define RXB1D0	    0x76 // Receive Buffer 1 Data Byte 0

//MCP2515 Command Bytes
#define RESET		    0xC0	// Reset instruction
#define READ			0x03	// Read instruction
#define READ_RX_BUFFER	0x90	// Read Receive Buffer
#define WRITE			0x02	// Write instruction
#define LOAD_TX_BUFFER	0x40	// Load transmission buffer
#define RTS				0x80	// Request to Send instruction
#define READ_STATUS		0xA0	// Read status bits
#define RX_STATUS		0xB0	// Receive status bits
#define BIT_MODIFY		0x05	// Bit modify instruction