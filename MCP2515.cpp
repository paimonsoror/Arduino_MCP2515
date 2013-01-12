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

#include "Arduino.h"
#include "SPI.h"
#include "MCP2515.h"

/** 
 * Initialize the MCP2515 library
 * @param	baudConst	The baud rate of the MCP2515 object
 * @param	ss			The slave select pin
 * @return				True if MCP2515 configured properly
 */
boolean MCP2515::initCAN(const int& baudConst, const int& ss)
{
  byte mode;
  SLAVESELECT = ss;
  
  Serial.print("Slave Select For Chipset ");
  Serial.println(SLAVESELECT);
  
  digitalWrite(SLAVESELECT,LOW);
  delay(10);
  
  // Perform an SPI Reset
  // Per section 10.1 of the reference this will automatically
  // place the device in configuration mode
  SPI.transfer(RESET);
  digitalWrite(SLAVESELECT,HIGH);
  delay(10);
  
  // Read mode and make sure it is config just to be
  // positive before we start sending configuration 
  // commands
  delay(100);
  mode = readReg(CANSTAT) >> 5;
  
  // If the mode doesn't equal configuration mode, try to force
  // the mode via SPI commands.
  if(mode != 0b100) {
	  
	  // If we still have trouble setting config mode, then we 
	  // have a problem with the device, and do not continue
	  if(!setMode(CONFIGURATION)) {
		Serial.print("Failed to set chip mode.  Mode is ");
		Serial.println(mode,BIN);
		return false;
	  }	
  }	  
  
  // Set the baud and return
  Serial.println("Setting Baud");
  return(setCANBaud(baudConst));
}

/** 
 * Set the CAN Baud Rate
 * @param	baudConst	The baud rate to set
 * @return				True if baud rate set
 */
boolean MCP2515::setCANBaud(const int& baudConst)
{
  byte brp;
  
  //BRP<5:0> = 00h, so divisor (0+1)*2 for 125ns per quantum at 16MHz for 500K   
  //SJW<1:0> = 00h, Sync jump width = 1
  switch(baudConst)
  {
    case CAN_BAUD_500K: brp = 0; break;
    case CAN_BAUD_250K: brp = 1; break;
    case CAN_BAUD_125K: brp = 3; break;
    case CAN_BAUD_100K: brp = 4; break;
    default: Serial.println("Non-Supported Baud Requested"); return false;
  }
  
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(WRITE); 
  SPI.transfer(CNF1);
  SPI.transfer(brp & 0b00111111);
  digitalWrite(SLAVESELECT,HIGH);  
  
  //PRSEG<2:0> = 0x01, 2 time quantum for prop
  //PHSEG<2:0> = 0x06, 7 time constants to PS1 sample
  //SAM = 0, just 1 sampling
  //BTLMODE = 1, PS2 determined by CNF3
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(WRITE); 
  SPI.transfer(CNF2);
  SPI.transfer(0b10110001);
  digitalWrite(SLAVESELECT,HIGH); 
  
  //PHSEG2<2:0> = 5 for 6 time constants after sample
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(WRITE); 
  SPI.transfer(CNF3);
  SPI.transfer(0x05);
  digitalWrite(SLAVESELECT,HIGH); 
  
  return true;
}

/** 
 * Set the operation mode of the MCP2515
 * @param	canMode	Enumeration that defines the requested mode
 * @return			True if the mode was successfully set
 */
boolean MCP2515::setMode(const CanMode& canMode) {
	byte mode, val, expected;
	
	// Register 10-1: CANCTRL
	// bit 7 - 5 : Request Operation Mode Bits
	// bit 4     : Abort all pending transmissions
	// bit 3     : One shot mode
	// bit 2     : CLKOUT Pin Enable
	// bit 1 - 0 : CLKOUT Pin Prescaler
	//
	// In each of these settings, we want to make sure that
	// we disable the flag that will reset all pending buffers
	// and we allow the messages to re-transmit if required
	// The rest can retain their values, so retain the old value
	switch(canMode) {
		case NORMAL:
			val = 0b00000111;
			expected = 0;
			break;
			
		case CONFIGURATION:
			val = 0b10000111;
			expected = 4;
			break;
			
		case LISTEN:
			val = 0b01100111;
			expected = 3;
			break;
		
		default:
			val = 0;
			expected = 0;
			break;		  
	};
	
	writeReg(CANCTRL, val);
	
	// Make sure we set the mode properly
	mode = readReg(CANSTAT) >> 5;
	if(mode != expected)
		return false;
		
	return true; 			
}	

/** 
 * Receive a message from the MCP2515
 * @param	msg		Structure to store the received data from
 * @param	timeout	Timeout for a response
 * @return			True if a message was read
 */
boolean MCP2515::receiveCANMessage(Frame& msg, const unsigned long& timeout)
{
    unsigned long endTime;
    boolean msgInB0;
    boolean msgInB1;
    boolean msgRxd;
    byte readVal;
    int byteIndex;
    
	// Define our end time by adding our timeout to the current time  
    endTime = millis() + timeout;
	
	// By default, we expect that we got a message
    msgRxd = true;
    msgInB0 = false;
    msgInB1 = false;
    
    //Wait for data to be received in either buffer
    while (millis() < endTime) {
        readVal = readReg(CANINTF);
        
		// Check what buffer we got the information from
        if (bitRead(readVal, RX0IF) == 1) {
            msgInB0 = true;
            break;
        } else if (bitRead(readVal, RX1IF) == 1) {
            msgInB1 = true;
            break;
        } else {
			msgRxd = false;
			return false;
		}
    }
    
    if (msgRxd) {
        //Was this data requested, is Remote Transmission Request (RTR) set?
        if (msgInB0) {
            readVal = readReg(RXB0CTRL);
        } else {
            readVal = readReg(RXB1CTRL);
        }
		
        msg.rtr = ((bitRead(readVal,3) == 1) ? true : false);
        
        // Start receiving buffer 0 from the SID high byte if
		// a message came from there, if not, read from the RX 1
		// SID high byte
        digitalWrite(SLAVESELECT,LOW);
        
        if (msgInB0) {
            SPI.transfer(READ_RX_BUFFER | READ_RX_0_SIDH);
        } else if (msgInB1) {
            SPI.transfer(READ_RX_BUFFER | READ_RX_1_SIDH);
        }
        
        // Read the response address from the bus
        msg.adrsValue = 0;
        readVal = SPI.transfer(0);
        msg.adrsValue = (readVal << 3);
        readVal = SPI.transfer(0);
        msg.adrsValue |= (readVal >> 5);
        
        // Check if the message returned is an extended CAN message,
		// if so, read the rest of the message
        msg.isExtendedAdrs = ((bitRead(readVal,EXIDE) == 1) ? true : false);
        msg.extendedAdrsValue = 0;
        if (msg.isExtendedAdrs) {
            msg.extendedAdrsValue = (readVal & 0x03) << 16;
            readVal = SPI.transfer(0);
            msg.extendedAdrsValue |= (readVal << 8);
			readVal = SPI.transfer(0);
			msg.extendedAdrsValue |= readVal;
        } else {
            SPI.transfer(0);
            SPI.transfer(0);
        }
        
        //Get the number of bytes received        
        readVal = SPI.transfer(0);
        msg.dataLength = (readVal & 0x0F); 
        if (msg.dataLength > 8) {
            msg.dataLength = 8;
        }
        
        //Read the actual data
        for (byteIndex = 0; byteIndex < msg.dataLength; byteIndex++) {
            msg.data[byteIndex] = SPI.transfer(0);
        }
        
        //End the communication with the chip. No need to reset the RXIF.
        digitalWrite(SLAVESELECT,HIGH);
    }
    
    return msgRxd;
}

/** 
 * Transmit a CAN message across the bus
 * @param	msg		The message structure to send
 * @param	timeout	Timeout for the message transmission
 * @return			True if message transmitted successfully
 */
boolean MCP2515::transmitCANMessage(Frame& msg, const unsigned long& timeout)
{
  boolean sentMessage = false;
  unsigned short val;
  
  // get our end time
  unsigned long endTime = millis() + timeout;

  // Read the address of the message we are about
  // to send
  val = msg.adrsValue >> 3;
  writeReg(TXB0SIDH,val);
  
  // Next, check to see if the message is an extended
  // message, if it is, shift so that we can get the 
  // lower bytes of the address
  val = msg.adrsValue << 5;
  if(msg.isExtendedAdrs)
    val |= 1 << EXIDE;
  writeReg(TXB0SIDL,val);
  
  // Was the message extended?  If so, store the message
  // in the extended register
  if(msg.isExtendedAdrs)
  {
    val = msg.extendedAdrsValue >> 8;
    writeReg(TXB0EID8,val);
    val = msg.extendedAdrsValue;
    writeReg(TXB0EID0,val);
  }
  
  // Set the DLC register with the value of the RTR
  val = msg.dataLength & 0x0f;
  if(msg.rtr)
    bitWrite(val,TXRTR,1);
  writeReg(TXB0DLC,val);
  
  // Set the message bytes
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(WRITE); 
  SPI.transfer(TXB0D0);
  for(int i = 0; i < msg.dataLength; i++)
    SPI.transfer(msg.data[i]);
  digitalWrite(SLAVESELECT,HIGH);

  // Transmit the message
  writeRegBit(TXB0CTRL,TXREQ,1);

  // As long as we haven't reached out timeout submit the
  // message and retry until the message is sent
  while(millis() < endTime)
  {
    val = readReg(CANINTF);
    if(bitRead(val,TX0IF) == 1)
    {
      sentMessage = true;
      break;
    }
  }

  //Abort the send if failed
  writeRegBit(TXB0CTRL,TXREQ,0);
  
  //And clear write interrupt
  writeRegBit(CANINTF,TX0IF,0);

  return sentMessage;
}

/** 
 * Get the transmission error count
 * @return	Total errors thrown on transmission
 */
byte MCP2515::getCANTxErrCnt()
{
  return(readReg(TEC));
}

/** 
 * Get the receive error count
 * @return	Total errors thrown on receive
 */
byte MCP2515::getCANRxErrCnt()
{
  return(readReg(REC));
}

/** 
 * Write a data byte to a register
 * @param	regno	The register to write to
 * @param	val		The byte to write to register
 */
void MCP2515::writeReg(const byte& regno, const byte& val)
{
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(WRITE); 
  SPI.transfer(regno);
  SPI.transfer(val);
  digitalWrite(SLAVESELECT,HIGH);  
}

/** 
 * Modify a bit in a register
 * @param	regno	Register to modify
 * @param	bitno	Bit in register to modify
 * @param   val		The byte to set
 */
void MCP2515::writeRegBit(const byte& regno, const byte& bitno, const byte& val)
{
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(BIT_MODIFY); 
  SPI.transfer(regno);
  SPI.transfer(1 << bitno);
  if(val != 0)
    SPI.transfer(0xff);
  else
    SPI.transfer(0x00);
  digitalWrite(SLAVESELECT,HIGH);
}

/** 
 * Read the value of a register
 * @param	regno	The register to read
 * @return			The byte value of the register
 */
byte MCP2515::readReg(const byte& regno)
{
  byte val;
  
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(READ); 
  SPI.transfer(regno);
  val = SPI.transfer(0);
  digitalWrite(SLAVESELECT,HIGH);
  
  return val;  
}  

/** 
 * Creates a message structure that complies with the OBD2
 * standards.  The message is sent across the bus and waits
 * for a response to come back.  The value is then returned
 * as well as the message is updated in memory
 * @param	code		The OBD2 code to request
 * @param	msg			The message response stored
 * @param   readSuccess True if read was successful
 * @param	bits		The total bits in the message
 * @return				The value returned from the request
 */
long MCP2515::queryOBD(const byte& code, Frame& msg, bool& readSuccess, const byte& bits)
{
  long val = 0;

  // Construct our OBD message
  msg.adrsValue = 0x7DF;
  msg.isExtendedAdrs = false;
  msg.extendedAdrsValue = 0;
  msg.rtr = false;
  msg.dataLength = 8;
  msg.data[0] = 0x02;
  msg.data[1] = 0x01;
  msg.data[2] = code;
  msg.data[3] = 0;
  msg.data[4] = 0;
  msg.data[5] = 0;
  msg.data[6] = 0;
  msg.data[7] = 0;
  
  // Submit the message, if our transmit wasn't succesful, just return
  if(!transmitCANMessage(msg,250))
    return 0;

  // Transmit was successful, now receive the response
  readSuccess = receiveCANMessage(msg,250);
  if (readSuccess) 
  {
    //Check if the PIDs match (in case other messages are also on bus)
	int noMatch = 0;
    while(msg.data[2] != code)
	{
		// If the PIDs didn't match, then retry until we get the response,
		// or until we have tried 5 times
        readSuccess = receiveCANMessage(msg,250);
        noMatch++;
        if (!readSuccess || noMatch >= 5) 
		{
            return 0;
        }
    }
  } 
  else 
    return 0;
  
  // Depending on the bits we are expecting from the
  // message, we need to manipulate the return data
  if(bits == 0) {
	// do nothing
  } else if(bits == 1) {
    val = msg.data[3];
  } else {
    val = 256 * msg.data[3] + msg.data[4];
  }	
    
  return val;
}

/** 
 * Query the ECU for extended information about the codes that are passed in.  We can supply an expected
 * response address as well as check the PID if necessary
 * @param expectedResp	The expected ECU address
 * @param msg			The message object
 */
boolean MCP2515::queryOBDExtended(const unsigned int& expectedResp, Frame& msg) {

  if(!transmitCANMessage(msg,CANTIMEOUT))
    return 0;

  boolean rxSuccess = receiveCANMessage(msg,CANTIMEOUT);

  if (rxSuccess) {
    //Check if the PIDs match (in case other messages are also on bus)
	int noMatch = 0;
	
	// If we didn't receive the correct return pid, keep checking
    while(msg.adrsValue != expectedResp) {
        rxSuccess = receiveCANMessage(msg, CANTIMEOUT);
        noMatch++;
        if (noMatch >= 5) {
            return 0;
        }
    }
	
	// If we got here, we assume we matched
	return 1;
  } 
  else 
    return 0;
}

/**
* Set receive filters and mask for both buffers
* 
* Only handles standard IDs. It can potentially handle all ID types
* relatively changes but I don't need that support.
* 
* Arguments
* buffer -- Which receive buffer's filters should be set? Possible values:
*              BUFFER_0 - Buffer 0
*              BUFFER_1 - Buffer 1
*              BUFFER_ALL - Buffer 0 and Buffer 1
* bufferFilter0 -- FILTER0 item that contains the mask and filters for
*                  receive buffer 0. If only buffer 1 is being set, 
*                  this is ignored.
* bufferFilter1 -- FILTER1 item that contains the mask and filters for
*                  receive buffer 1. If only buffer 0 is being set, 
*                  this is ignored.
* 
* Returns
* Boolean indicating whether the given filters were set
* 
* Added by Christopher Meyer July, 2011
* 
*/
boolean MCP2515::setReceiveFilter(unsigned int buffer, FILTER0 bufferFilter0,
	FILTER1 bufferFilter1)
{
    unsigned short maskAddr;
    unsigned short mask;
    byte filterAddr;
    unsigned short filterCnt;
    unsigned short activeFilters[4];
    byte bufferCtl;
    unsigned short addressOffset;
    unsigned short i;
    
    if (buffer == BUFFER_ALL) {
        if (!setReceiveFilter(BUFFER_0, bufferFilter0, bufferFilter1)) {
            return false;
        }
        buffer = BUFFER_1;
    }

    if (buffer == BUFFER_0) {
        maskAddr = RXM0SIDH;
        mask = bufferFilter0.mask;
        filterAddr = RXF0SIDH;
        filterCnt = 2;
        activeFilters[0] = bufferFilter0.filters[0];
        activeFilters[1] = bufferFilter0.filters[1];
        bufferCtl = RXB0CTRL;
        
    } else {
        maskAddr = RXM1SIDH;
        mask = bufferFilter1.mask;
        filterAddr = RXF2SIDH;
        filterCnt = 4;
        activeFilters[0] = bufferFilter1.filters[0];
        activeFilters[1] = bufferFilter1.filters[1];
        activeFilters[2] = bufferFilter1.filters[2];
        activeFilters[3] = bufferFilter1.filters[3];
        bufferCtl = RXB1CTRL;
    }

    //Set the mask RXM0
	digitalWrite(SLAVESELECT,LOW);
	SPI.transfer(WRITE); 
	SPI.transfer(maskAddr);
	SPI.transfer(mask >> 3);
	SPI.transfer((mask & 0xFF) << 5);
	digitalWrite(SLAVESELECT,HIGH);
    
	//Verify mask was set
	if (readReg(maskAddr) != (mask >> 3)) {
		return false;
	}

	//Set filters
    digitalWrite(SLAVESELECT,LOW);
    SPI.transfer(WRITE); 
    SPI.transfer(filterAddr);
    for (i = 0; i < filterCnt; i++) {
        if ((buffer == BUFFER_1) && (i == 1)) {
            //The filters for buffer 1 are not contiguous
            digitalWrite(SLAVESELECT,HIGH);
            digitalWrite(SLAVESELECT,LOW);
            SPI.transfer(WRITE); 
            SPI.transfer(RXF3SIDH);
        }
        SPI.transfer(activeFilters[i] >> 3);
        SPI.transfer((activeFilters[i] & 0xFF) << 5);
        // The EID filters are unused so just set them to zero
        SPI.transfer(0);
        SPI.transfer(0);
    }
    digitalWrite(SLAVESELECT,HIGH);

	// Verify the filters were set
    addressOffset = 0;
	for (i = 0; i < filterCnt; i++) {
        if ((buffer == BUFFER_1) && i == 1) {
            //Add an offset for filters RXF3 and up (buffer 1 filters 1 and up)
            addressOffset = 4;
        }
        
        if (readReg(filterAddr + 4 * i + addressOffset) != 
            (activeFilters[i] >> 3)) {
            return false;
        }
    }

	//Set receive operating mode to SID filter mode
	writeRegBit(bufferCtl, RXM1, 0);
	writeRegBit(bufferCtl, RXM0, 1);

	//Verify that RXM in the buffer control register was set to filter SIDs
	if ((readReg(bufferCtl) & 0x60) != 0x20) {
		return false;
	}

	return true;
}

/**
 * Set the receive filters to only accept data from a specific range
 * of addresses
 * @return	True if the filters were properly set
 */
boolean MCP2515::setOBDFilters()
{
    FILTER0 obdFilter0;
    FILTER1 obdFilter1;
	
    obdFilter0.mask = 0x650;
    obdFilter0.filters[0] = 0x650;
    obdFilter0.filters[1] = 0x650;
    
    obdFilter1.mask = 0x7FF;       // 111 1111 1111
    obdFilter1.filters[0] = 0x7E8; // 111 1110 1000
    obdFilter1.filters[1] = 0x759; // 111 0101 1001
    obdFilter1.filters[2] = 0x650; // 110 0101 0000
    obdFilter1.filters[3] = 0x4B0; // 100 1011 0000
    
	return (setReceiveFilter(BUFFER_ALL, obdFilter0, obdFilter1));
}