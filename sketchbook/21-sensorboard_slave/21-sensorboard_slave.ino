/*
 * 21-sensorboard_slave.ino
 * Wesley T. Honeycutt; 2021
 * 
 * License: GPL-3.0
 * 
 * This code is for the "CONEX Sensor Deployment" with the OU School of Architecture.
 * 
 * This program controls a small breakout board (CO2x sensorboard_slave) with an Arduino nano
 * controller.  This breakout board is attached to a modified sensorboard (CO2x sensorboard) from
 * Dr. Honeycutt's 2017 Dissertation work.  The logic of this code waits for a command from a
 * master node to perform any actions.  Actions include power on 5V lines, power off 5V lines,
 * read sensors and respond.  Boards communicate by a custom byte struct which includes a command
 * header, payload, and CRC32 checksum.  For the analog MQ4 sensor, this code uses a clever trick
 * to get a more accurate reference voltage from the Arduino.
 * 
 * Instructions:
 * Prior to use, the user should ensure that each device in the network recieves a unique ID value
 * coded in DEV_ID.  The units programmed using this slave code must start at 0x31 and go up.  The
 * master node reserves 0x30.  Skipping values is not advised, since that will create delay in the
 * command sequence for the master node.  Keep it simple: 0x31, 0x32, 0x33 and so on.
 * 
 * Required modifications to sensorboard:
 * - R11, R12 remove
 * - R13, R14 change to 0\Omega resistors
 * - R15, R16 change to 2.2k\Omega resistors
 * - X1 (PCA9517) remove and connect pad pairs 2/7 and 3/6 with wires.
 */


#include <Wire.h>
#include <SoftEasyTransfer.h>
#include <SoftwareSerial.h>
#include <CRC32.h>
#include "Sensirion.h"


// ALERT This must be unique for each unit on the network!
// This device ID
const byte DEV_ID = 0x32; // Ox30 reserved for master

// Magic numbers
const byte STARTCODON = 0x01;
const byte ENDCODON   = 0x04;
const byte B_XON      = 0x11;
const byte B_XOFF     = 0x13;
const byte B_ENQ      = 0x05;
const byte B_RESP     = 0x43;
const byte B_ACK      = 0x06;
const byte B_NACK     = 0x15;
const byte B_MASTER   = 0x30;

const int MSG_SIZE = 25;
uint8_t PREV_CMD = 0;
bool RECV_CMD = false;
bool SEND_CMD = false;
bool MY_MSG = false;

struct CMD_BYTE_BUFFER {
	byte msg[MSG_SIZE] = {
 		0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
 		0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
 		0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
 		0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
};
CMD_BYTE_BUFFER cmddata;

// Declaire pin values for SHT7x sensor
const uint8_t dataPin  = 10;
const uint8_t clockPin =  9;

// Declare pin values for CO2 sensor
// These are wired to A4 and A5, the default i2c pins.  We don't need to declare them.

// Declare pin values for MPX sensor
const uint8_t mpxPin   = 11;

// Declare pin value for MQ-4 sensor
const uint8_t ch4Pin   = A0;

// Declare power switch pins
const uint8_t ch4_en   =  7; // MQ-4
const uint8_t bd_en    = 12; // Everything else on sensorboard
const uint8_t pump_en  = 13; // Air pump

// Declare pin value of our "Transmit Enable" for the RS485 connection
const uint8_t RS485_TE =  6;

// init SHT7x sensor, does not power it on yet
Sensirion tempSensor = Sensirion(dataPin, clockPin); //(data, clk)

// Address of CO2 sensor. This is the default address of the CO2 sensor, 7bits shifted left.
int co2Addr = 0x68;

// init our software serial as RX = 2, TX = 3 (This is the RS485)
SoftwareSerial mySerial(3, 2);

// Float handler
typedef union {
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;
FLOATUNION_t myFloat;

// Declare output values.  These may be volatile, but no one else touches them, so we do not 
// include the "volatile" qualifier.
float tval;
float rhval;
float dpval;
float chval;
float pval;
float my_vcc;
uint16_t co2_val;

uint16_t readCO2() {
	/*
	 * This function requests the CO2 concentration from the K30 meter.
	 * 
	 * Returns int value or 0
	 */
	// init empty CO2 value
	uint16_t co2_value = 0;
	// init checksum
	byte sum = 0;
	// init byte buffer to read into and the counter
	byte i = 0;
	byte buffer[4] = {0, 0, 0, 0};
	// Write to the co2Addr some byte codes which command it to read the CO2 value
	Wire.beginTransmission(co2Addr);
	Wire.write(0x22);
	Wire.write(0x00);
	Wire.write(0x08);
	Wire.write(0x2A);
	Wire.endTransmission();
	// Wait for the RAM to clear
	delay(25);
	// Read the bytes from the sensor (4 bytes: 2 byte payload, checksum, and status)
	Wire.requestFrom(co2Addr, 4);
	// Read this into our buffer
	while (Wire.available()) {
		buffer[i] = Wire.read();
		i++;
	}
	for (size_t i = 0; i < sizeof(buffer); i++) {
		Serial.print(buffer[i]);
		Serial.print(" ");
	}
	Serial.println();
	// Convert buffer to int
	co2_value = 0;
	co2_value |= buffer[1] & 0xFF;
	co2_value = co2_value << 8;
	co2_value |= buffer[2] & 0xFF;
	// compare the checksum.  Byte addition utilizes overflow.
	sum = buffer[0] + buffer[1] + buffer[2];
	if (sum == buffer[3]) {
		// Success!
		return co2_value;
	} else {
		// Failure!
		return 0;
	}
}


int readVcc1() {
	/*
	 * This function returns the input voltage by using an internal comparision with the 1.1V AREF
	 * without requiring additional external components.  It is not documented by AVR, presumably
	 * because it is not super-good.  It gets us to a better response than assuming 5V though, since
	 * the many devices on this network and board pull the Vin closer to 4.5V.
	 * 
	 * Code borrowed from:
	 * https://arduino.stackexchange.com/questions/23526/measure-different-vcc-using-1-1v-bandgap
	 * 
	 * return: actual value of Vcc (x 100)
	 */
	uint8_t oldADMUX = ADMUX;
	const long InternalReferenceVoltage = 1100L; // Adjust this value to your boards specific internal BG voltage x1000

	#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
		// For mega boards
		// REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc reference
		// MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)         -Selects channel 30, bandgap voltage, to measure
	  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (0 << MUX5) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
	#else
		// For 168/328 boards
		// REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
		// MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
		ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
	#endif
	delay(50); // Let mux settle a little to get a more stable A/D conversion
	// Start a conversion
	ADCSRA |= _BV(ADSC);
	// Wait for it to complete
	while (((ADCSRA & (1 << ADSC)) != 0));
	// Scale the value linearly
	int results = (((InternalReferenceVoltage * 1023L) / ADC) + 5L) / 10L;
	// revert value
	ADMUX = oldADMUX;
	Serial.print("Vin = ");
	Serial.println(results);

	return results;
}

void fetch_data() {
	/*
	 * Fetch the data from all the sensors and store them in the globals
	 */
	// Current Vcc
	my_vcc = ((float)readVcc1())/100;
	// SHT7x data
	Serial.print("pre t= ");
	Serial.println(tval);
	tempSensor.measure(&tval, &rhval, &dpval);
	Serial.print("post t= ");
	Serial.println(tval);
	// Pressure - requires ADC conversion.  Outputs torr.
	Serial.print("pre p= ");
	Serial.println(pval);
	pval = analogRead(mpxPin);
	pval = 833.4*(((pval/1024) * (5.0/my_vcc)) + 0.095);  //Materer version in torr
	Serial.print("post p= ");
	Serial.println(pval);
	// CH4 - requires ADC conversion vs a 10k resistor
	Serial.print("pre ch= ");
	Serial.println(chval);
	chval = analogRead(ch4Pin);
	chval = (chval/1024) * (5.0/my_vcc);
	chval = ((5.0 - chval)/chval/10000);
	Serial.print("post ch= ");
	Serial.println(chval);
	
	// CO2
	Serial.print("pre co= ");
	Serial.println(co2_val);
	co2_val = readCO2();
	Serial.print("post co= ");
	Serial.println(co2_val);
	return;
}


void package_data() {
	/*
	 * Package the data stored in the globals into a byte string.
	 */
	
	/*
	 * My byte string format
	 * byte 0 - start of message
	 * byte 1 - recipient address
	 * byte 2 - command
	 * byte 3-6 - payload (t)
	 * byte 7-10 - payload (rh)
	 * byte 11-14 - payload (p)
	 * byte 15-18 - payload (ch4)
	 * byte 19-20 - payload (co2)
	 * byte 21-24 - crc32 checksum
	 * byte 25 - end codon
	 */
	
	/*
	 * byte 2 command list:
	 * ACK = 0x06 (message received)
	 * NACK = 0x15 (message failed, please resend)
	 * ENQ = 0x05 (wake up and report sensors)
	 * listen = 0x43
	 */
	
	fetch_data();
	
	cmddata.msg[0] = STARTCODON;
	cmddata.msg[1] = B_MASTER;
	cmddata.msg[2] = B_RESP;
	// T
	myFloat.number = tval;
	for (int i=0; i < 4; i++) {
		cmddata.msg[3 + i] = myFloat.bytes[i];
	}
	// Rh
	myFloat.number = rhval;
	for (int i=0; i < 4; i++) {
		cmddata.msg[7 + i] = myFloat.bytes[i];
	}
	// P
	myFloat.number = pval;
	for (int i=0; i < 4; i++) {
		cmddata.msg[11 + i] = myFloat.bytes[i];
	}
	// CH4
	myFloat.number = chval;
	for (int i=0; i < 4; i++) {
		cmddata.msg[15 + i] = myFloat.bytes[i];
	}
	// CO2
	cmddata.msg[19] = highByte(co2_val);
	cmddata.msg[20] = lowByte(co2_val);
	create_checksum();
	return;
}

void cmd_ack() {
	for (size_t i = 0; i < MSG_SIZE; i++) {
		cmddata.msg[i] = 0x30;
	}
	cmddata.msg[0] = STARTCODON; // SOH
	cmddata.msg[1] = B_MASTER; // ASCII "0" is master node
	cmddata.msg[2] = B_ACK; // ACK
	create_checksum();
	return;
}

void cmd_nack() {
	for (size_t i = 0; i < MSG_SIZE; i++) {
		cmddata.msg[i] = 0x30;
	}
	cmddata.msg[0] = STARTCODON; // SOH
	cmddata.msg[1] = B_MASTER; // ASCII "0" is master node
	cmddata.msg[2] = B_NACK; // NACK
	create_checksum();
	return;
}

void cmd_sensor_results() {
	// TODO remove junk data
	for (size_t i = 0; i < MSG_SIZE; i++) {
		cmddata.msg[i] = 0x30 + i;
	}
	cmddata.msg[0] = STARTCODON; // SOH
	cmddata.msg[1] = B_MASTER; // ASCII "0" is master node
	cmddata.msg[2] = B_RESP; // Response
	create_checksum();
	return;
}

void cmd_listener() {
	/*
	 * Non-blocking Serial reader
	 */
	static bool listening = false;
	static size_t cnt = 0;
	char in_char;
	
	while (mySerial.available() > 0 && RECV_CMD == false) {
		in_char = mySerial.read();
		Serial.print(in_char, HEX);
		if (listening == true) {
			if (in_char != ENDCODON) {
				cmddata.msg[cnt] = in_char;
				cnt++;
				// Handle overruns
				if (cnt >= MSG_SIZE) {
					cnt = MSG_SIZE - 1;
				}
			}
			else {
				cmddata.msg[cnt] = ENDCODON;
				listening = false;
				cnt = 0;
				Serial.println();
				RECV_CMD = true;
			}
		}

		else if (in_char == STARTCODON) {
			listening = true;
			cmddata.msg[cnt] = in_char;
			cnt++;
		}
	}
	
	return;
}

void create_checksum() {
	// Checksum
	CRC32 crc;
	for (size_t i = 0; i < (MSG_SIZE - 4); i++) {
		crc.update(cmddata.msg[i]);
	}
	uint32_t checksum = crc.finalize();
	cmddata.msg[21] = (checksum >> 0)  & 0xFF;
	cmddata.msg[22] = (checksum >> 8)  & 0xFF;
	cmddata.msg[23] = (checksum >> 16) & 0xFF;
	cmddata.msg[24] = (checksum >> 32) & 0xFF;
	return;
}

void cmd_sender() {
	if (SEND_CMD) {
		digitalWrite(RS485_TE, HIGH);
		for (size_t i=0; i<sizeof(cmddata.msg); i++) {
 			mySerial.write(cmddata.msg[i]);
 		}
		mySerial.write(ENDCODON);
		mySerial.flush();
		digitalWrite(RS485_TE, LOW);
		SEND_CMD = false;
	}
	return;
}

void nack_handler() {
	if (PREV_CMD == 1) {
		package_data();
	} else if (PREV_CMD == 2) {
		digitalWrite(ch4_en, HIGH);
// 		digitalWrite(bd_en, HIGH);
		digitalWrite(pump_en, HIGH);
// 		cmd_ack();
	} else if (PREV_CMD == 3) {
		digitalWrite(ch4_en, LOW);
// 		digitalWrite(bd_en, LOW);
		digitalWrite(pump_en, LOW);
// 		cmd_ack();
	} else {
		cmd_nack();
	}
	SEND_CMD = true;
	return;
}

void proc_cmd() {
	if (RECV_CMD) {
		Serial.println("Recieved cmd");
		
		// Checksum
		CRC32 crc;
		for (size_t i = 0; i < (MSG_SIZE - 4); i++) {
			crc.update(cmddata.msg[i]);
		}
		uint32_t checksum = crc.finalize();
		if ((cmddata.msg[21] != ((checksum >> 0) & 0xFF))
			&& (cmddata.msg[22] != ((checksum >> 8)  & 0xFF))
			&& (cmddata.msg[23] != ((checksum >> 16) & 0xFF))
			&& (cmddata.msg[24] != ((checksum >> 32) & 0xFF))) {
			Serial.println("CRC FAIL");
			
			cmd_nack();
			SEND_CMD = true;
			RECV_CMD = false;
			return;
		}
		
		// Check if this command was intended for this unit
		if (cmddata.msg[1] == DEV_ID) {
			MY_MSG = true;
		}
		
		// Interpret message types if it was intended for this unit
		if (MY_MSG) {
			if (cmddata.msg[2] == B_ENQ) {
				PREV_CMD = 1;
				package_data();
				SEND_CMD = true;
				
			} else if (cmddata.msg[2] == B_ACK) {
				// Do nothing
			} else if (cmddata.msg[2] == B_NACK) {
				nack_handler();
				SEND_CMD = true;
// 				RECV_CMD = false;
			} else if (cmddata.msg[2] == B_XON) {
				PREV_CMD = 2;
				digitalWrite(ch4_en, HIGH);
// 				digitalWrite(bd_en, HIGH);
				digitalWrite(pump_en, HIGH);
				cmd_ack();
				SEND_CMD = true;
			} else if (cmddata.msg[2] == B_XOFF) {
				PREV_CMD = 3;
				digitalWrite(ch4_en, LOW);
// 				digitalWrite(bd_en, LOW);
				digitalWrite(pump_en, LOW);
				cmd_ack();
				SEND_CMD = true;
			} else {
				// Recieved nonsense command
				cmd_nack();
				SEND_CMD = true;
			}
			MY_MSG = false;
		}
		
		RECV_CMD = false;
	}
	return;
}

void setup() {
	Serial.begin(9600);
	Serial.print("Debugging: ");
	Serial.println(__FILE__);
	
	Wire.begin();
	
	// Init the software serial
	pinMode(RS485_TE, OUTPUT);
	digitalWrite(RS485_TE, LOW);
	mySerial.begin(9600);
// 	ET.begin(details(msgdata), &mySerial);
	
// 	mySerial.flush();
	
	pinMode(ch4_en, OUTPUT);
	digitalWrite(ch4_en, LOW);
	pinMode(bd_en, OUTPUT);
	digitalWrite(bd_en, HIGH);
	pinMode(pump_en, OUTPUT);
	digitalWrite(pump_en, LOW);
	
	Serial.println("setup complete");
}

void loop() {
// 	Serial.println("loop");
	
	proc_cmd();
	
	cmd_listener();
	cmd_sender();
	
// 	proc_command();
// 	
// 	msg_listener();
// 	msg_sender(); // must come AFTER cmd_sender
}
