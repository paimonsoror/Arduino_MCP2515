#ifndef MCP2515_h
#define MCP2515_h

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
 ************************************************************************/

#include "Arduino.h"
#include "MCP2515_defs.h"

// Data rate selection constants
#define CAN_BAUD_10K 1
#define CAN_BAUD_50K 2
#define CAN_BAUD_100K 3
#define CAN_BAUD_125K 4
#define CAN_BAUD_250K 5
#define CAN_BAUD_500K 6

// Define the CAN timeout
#define CANTIMEOUT 100

/** 
 * Structure that defines a CAN frame
 */
typedef struct
{
  unsigned short adrsValue;
  boolean isExtendedAdrs;
  unsigned long extendedAdrsValue;
  boolean rtr;
  byte dataLength;
  byte data[8];
}  Frame;

/** 
 * Enumeration that defines the operation mode for
 * the MCP chip
 */
enum CanMode {NORMAL,		 // Normal Mode
			  CONFIGURATION, // Configuration Mode
			  LISTEN};       // Listen only mode
			  
/* Only handles Standard IDs
 * Mask - Mask of the bits that are to be filtered, 1 - Check, 0 - Ignore
 * Filters - What values the unmasked bits should be. When a message matches,
 *              it will be passed onto RX buffer 0. 
 * There are 2 filters for RX 0.
 */
typedef struct {
    unsigned short mask;
    unsigned short filters[2];
} FILTER0;

/* Only handles Standard IDs
 * Mask - Mask of the bits that are to be filtered, 1 - Check, 0 - Ignore
 * Filters - What values the unmasked bits should be. When a message matches,
 *              it will be passed onto RX buffer 1.
 * There are 4 filters for RX 1.
 */
typedef struct {
    unsigned short mask;
    unsigned short filters[4];
} FILTER1;

/** 
 * MCP2515 Class object
 */
class MCP2515
{
	public:
		boolean initCAN(const int& baudConst, const int& ss);
		boolean setMode(const CanMode& canMode);
		boolean receiveCANMessage(Frame& msg, const unsigned long& timeout);
		boolean transmitCANMessage(Frame& msg, const unsigned long& timeout);
		byte getCANTxErrCnt();
		byte getCANRxErrCnt();
		
		long queryOBD(const byte& code, Frame& msg, bool& readSuccess, const byte& bits = 0);
		boolean queryOBDExtended(const unsigned int& expectedResp, Frame& msg);
		
		boolean setReceiveFilter(unsigned int buffers, FILTER0 filter0,	FILTER1 filter1);
		boolean setOBDFilters();
	
	private:
		boolean setCANBaud(const int& baudConst);
		void writeReg(const byte& regno, const byte& val);
		void writeRegBit(const byte& regno, const byte& bitno, const byte& val);
		byte readReg(const byte& regno);
		
	private:
		int SLAVESELECT;
};

#endif