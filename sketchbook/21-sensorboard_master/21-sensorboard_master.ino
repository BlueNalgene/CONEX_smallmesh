/*
 * 21-sensorboard_master.ino
 * Wesley T. Honeycutt; 2021
 * 
 * License: GPL-3.0
 * 
 * This code is for the "CONEX Sensor Deployment" with the OU School of Architecture.
 * 
 * 
 */

#include <CRC32.h>
#include <SPI.h>
#include <SD.h>

#include "RTClib.h"

//************** magic numbers *****************//

// This block is used for communication control; based on xmodem
const byte STARTCODON = 0x01;
const byte ENDCODON   = 0x04;
const byte B_XON      = 0x11;
const byte B_XOFF     = 0x13;
const byte B_ENQ      = 0x05;
const byte B_RESP     = 0x43;
const byte B_ACK      = 0x06;
const byte B_NACK     = 0x15;
const byte B_MASTER   = 0x30;



//************** custom struct *****************//

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
 * byte 25 - end codon (added by line ending)
 */

/*
 * byte 2 command list:
 * ACK = 0x06 (message received)
 * NACK = 0x15 (message failed, please resend)
 * ENQ = 0x05 (wake up and report sensors)
 * listen = 0x43
 */

const int MSG_SIZE = 25;

struct CMD_BYTE_BUFFER {
	byte msg[MSG_SIZE] = {
 		0x40, 0x41, 0x42, 0x43, 0x44,
 		0x41, 0x42, 0x43, 0x44, 0x45,
 		0x42, 0x43, 0x44, 0x45, 0x46,
 		0x43, 0x44, 0x45, 0x46, 0x47,
		0x44, 0x45, 0x46, 0x47, 0x48};
};
CMD_BYTE_BUFFER cmddata;

// Declare pin value of our "Transmit Enable" for the RS485 connection
const uint8_t RS485_TE =  22;



//************** SD card log *****************//

File logFile;
char LOGFILENAME[] = "LOGXXXX.CSV";
// Counter for the number of lines written to the log file to know when we need to restart.
uint32_t CNT_LOG_LINES = 0;
/*
 * Max CNT_LOG_LINES allows before starting a new file. Should be well below 2gb limit for FAT.
 * Each line of the CSV is 72 bytes.
 * Assume max file size is 2147483648 Bytes (true 2GB).
 * Max lines is probably 29826160, fudge factored down for safety.
 * If data are reported every second, this will take ~290 days to fill with a single sensor unit.
 * It will be the same time for the planned 6 sensors if they cycle.
 */
const uint32_t MAX_LOG_LINES = 25000000;



//************** clock info *****************//

RTC_PCF8523 rtc;

// milliseconds between each "cycle" of the commands.  Since we do a lot of on and off, the true
// cycle time will be 3*CYC_INTV
const unsigned long CYC_INTV = 5000;

// This is a flag set when it is time to issue a new command to the network
bool CYC_SWT = false;

// This is a holder for any millisecond counters
unsigned long MIL_CNT = 0;



//************** unit cycler *****************//

uint8_t WHICH_UNIT = 0;
// ALERT the user should ensure this is the lowest value of device ID in the slave nodes.
// e.g. if the min value of slave units is 0x31, set MIN_UNIT = 1.
uint8_t MIN_UNIT = 1;
// ALERT the user should set this equal to their max number of nodes in the network.
// e.g. if the max value of slave units is 0x3F, set MAX_UNIT = 15.
uint8_t MAX_UNIT = 2;



//************** Status Flags *****************//

// Store the previous command in case we get a NACK
int LAST_CMD = 0;

// Did I recieve a message?
bool RECV_CMD = false;

// Am I ready to send a message?
bool SEND_CMD = false;




void cmd_listener() {
	/*
	 * Non-blocking Serial reader.  This listens to any message coming through the RS485
	 * connection and stores it in the data struct.  The code handles data overrun by crudely
	 * packing anything extra onto msg[24] if the message did not end as expected.  This may
	 * cause problems, but since msg[24] is part of the CRC checksum, the error will be caught
	 * elsewhere.
	 * 
	 * Sets RECV_CMD flag.
	 * 
	 * return: nothing
	 */
	static bool listening = false;
	static size_t cnt = 0;
	char in_char;
	
	while (Serial1.available() > 0 && RECV_CMD == false) {
		in_char = Serial1.read();
		// I am listening and checking for the EOF byte.
		if (listening == true) {
			if (in_char != ENDCODON) {
				cmddata.msg[cnt] = in_char;
				cnt++;
				// Handle overruns
				if (cnt >= MSG_SIZE) {
					cnt = MSG_SIZE - 1;
				}
			// I am no longer listening.  Shut down this loop.
			} else {
				cmddata.msg[cnt] = ENDCODON;
				listening = false;
				cnt = 0;
				RECV_CMD = true;
			}
		// If we just heard a SOH byte, keep this loop alive.
		} else if (in_char == STARTCODON) {
			listening = true;
			cmddata.msg[cnt] = in_char;
			cnt++;
		}
	}
	
	return;
}

void cmd_sender() {
	/*
	 * If this unit is ready to send a message, toggle the TE, send each byte sequentially from
	 * the struct, end with EOF, wait for everything to send, and finally disable the TE.
	 * 
	 * return: nothing
	 */
	if (SEND_CMD) {
		digitalWrite(RS485_TE, HIGH);
		for (size_t i=0; i<sizeof(cmddata.msg); i++) {
 			Serial1.write(cmddata.msg[i]);
 		}
 		// Manually write the EOF byte so we don't have to store it in the struct
		Serial1.write(ENDCODON);
		// Wait for everything to send
		Serial1.flush();
		digitalWrite(RS485_TE, LOW);
		SEND_CMD = false;
	}
	return;
}

void time_check() {
	/*
	 * Test the time this program has been running since the beginning of time (MIL_CNT).  If this
	 * has been running longer than our max time (CYC_INTV), then it is time to cycle the command
	 * by setting the CYC_SWT flag.  In this way we can accurately-ish time our commands.
	 * 
	 * return: nothing
	 */
	if (millis() > (MIL_CNT + CYC_INTV)) {
		CYC_SWT = true;
		MIL_CNT = millis();
	}
}

void cmd_ack() {
	/*
	 * Fill the command message struct with the ACKnowledged message.
	 * 
	 * return: nothing
	 */
	for (size_t i = 0; i < MSG_SIZE; i++) {
		cmddata.msg[i] = 0x30;
	}
	cmddata.msg[0] = STARTCODON; // SOH
	cmddata.msg[1] = B_MASTER + 1; // ASCII "0" is master node
	cmddata.msg[2] = B_ACK; // XON
	create_checksum();
	return;
}

void cmd_nack() {
	/*
	 * Fill the command message struct with the NotACKnowledged message.
	 * 
	 * return: nothing
	 */
	for (size_t i = 0; i < MSG_SIZE; i++) {
		cmddata.msg[i] = 0x30;
	}
	cmddata.msg[0] = STARTCODON; // SOH
	cmddata.msg[1] = B_MASTER + 1; // ASCII "0" is master node
	cmddata.msg[2] = B_NACK; // XON
	create_checksum();
	return;
}

void cmd_xon(uint8_t unit) {
	/*
	 * Fill the command message struct with the turnXON message.
	 * 
	 * return: nothing
	 */
	for (size_t i = 0; i < MSG_SIZE; i++) {
		cmddata.msg[i] = 0x30;
	}
	cmddata.msg[0] = STARTCODON; // SOH
	cmddata.msg[1] = B_MASTER + unit; // ASCII "0" is master node
	cmddata.msg[2] = B_XON; // XON
	create_checksum();
	return;
}

void cmd_xoff(uint8_t unit) {
	/*
	 * Fill the command message struct with the turnXOFF message.
	 * 
	 * return: nothing
	 */
	for (size_t i = 0; i < MSG_SIZE; i++) {
		cmddata.msg[i] = 0x30;
	}
	cmddata.msg[0] = STARTCODON; // SOH
	cmddata.msg[1] = B_MASTER + unit; // ASCII "0" is master node
	cmddata.msg[2] = B_XOFF; // XOFF
	create_checksum();
	return;
}

void cmd_enq(uint8_t unit) {
	/*
	 * Fill the command message struct with the ENQuire message.
	 * 
	 * return: nothing
	 */
	for (size_t i = 0; i < MSG_SIZE; i++) {
		cmddata.msg[i] = 0x30;
	}
	cmddata.msg[0] = STARTCODON; // SOH
	cmddata.msg[1] = B_MASTER + unit; // ASCII "0" is master node
	cmddata.msg[2] = B_ENQ; // ENQ
	create_checksum();
	return;
}

void prep_cmd() {
	/*
	 * Check if we need to prepare a command.  If so, do the next one in the order.
	 */
	if (CYC_SWT) {
		if (LAST_CMD == 0) {
			cmd_xon(WHICH_UNIT);
			LAST_CMD++;
		} else if (LAST_CMD == 1) {
			cmd_enq(WHICH_UNIT);
			LAST_CMD++;
		} else if (LAST_CMD == 2) {
			cmd_xoff(WHICH_UNIT);
			LAST_CMD = 0;
			WHICH_UNIT++;
		}
		SEND_CMD = true;
		CYC_SWT = false;
	}
	return;
}

void create_checksum() {
	/*
	 * Parse the header and payload regions of the data struct to generate a checksum.  Break the
	 * checksum into bytes for storage in the struct as well.
	 * 
	 * return: nothing
	 */
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

void decrease_cmd_iter() {
	/*
	 * In the event that we received a NACK or failed CRC check, we need to set the unit counter
	 * back by one so that we can address the issue.
	 * 
	 * return: nothing
	 */
	
	// If we had a CRC nack, set the command loop back by one.
	if (LAST_CMD > 0) {
		LAST_CMD--;
	} else {
		LAST_CMD = 2;
		if (WHICH_UNIT == MIN_UNIT) {
			WHICH_UNIT = MAX_UNIT;
		}
	}
}

void proc_cmd() {
	/*
	 * In the event that this unit recieved a command we process it here.  Since we know that
	 * messages are only addressed to this unit in the network, we don't have to worry about
	 * reading the address,  First, we check the CRC makes sense.  Next, we check the
	 * command byte and respond appropriately.
	 * 
	 * return: nothing
	 */
	if (RECV_CMD) {
		
		// Debugging
		for (size_t i = 0; i < (MSG_SIZE); i++) {
			Serial.print(cmddata.msg[i], HEX);
			Serial.print(" ");
		}
		Serial.println();
		
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
			
			decrease_cmd_iter();
			cmd_nack();
			SEND_CMD = true;
			RECV_CMD = false;
			return;
		}
		
		// If we were NotACKnowledged...
		if (cmddata.msg[2] == B_NACK) {
			Serial.println("NACK");
			decrease_cmd_iter();
		// If we received a data RESPonse
		} else if (cmddata.msg[2] == B_RESP) {
			save_data();
			cmd_ack();
			SEND_CMD = true;
		// If we were ACKnowledged...
		} else if (cmddata.msg[2] == B_ACK) {
			Serial.println("ACK");
		// If we recieved a nonsense command...
		} else {
			Serial.println("nonsense");
			cmd_nack();
			SEND_CMD = true;
		}
		RECV_CMD = false;
	}
	return;
}

void unit_looper() {
	/*
	 * If our list WHICH_UNIT value overruns the max in our network, revert.  This does not
	 * increment WHICH_UNIT, as that is handled in prep_cmd().  Instead, it only prevents an
	 * overrun.
	 * 
	 * return: nothing
	 */
	if (WHICH_UNIT > MAX_UNIT) {
		WHICH_UNIT = MIN_UNIT;
	}
	return;
}

void print_sd_dir() {
	/*
	 * Search the files on the SD card to find the one with the highest numeric match to the
	 * template LOG0000.csv.  Create a new file which is +1 to this old file.
	 * 
	 * return: nothing
	 */
	int16_t max_val_log = -1;
	
	File dir = SD.open("/");
	
	while (true) {
		File entry =  dir.openNextFile();
		if (! entry) {
			// no more files
			break;
		}
		char* name_buff = entry.name();
		
		if (strlen(name_buff) == 11) {
			if ((name_buff[0] != 'L')
				|| (name_buff[1] != 'O')
				|| (name_buff[2] != 'G')
				|| (name_buff[7] != '.')
				|| (name_buff[8] != 'C')
				|| (name_buff[9] != 'S')
				|| (name_buff[10] != 'V')) {
				continue;
			}
			// truncate first 3 chars
			char *num_array = &name_buff[3];
			// truncate after 4 chars
			num_array[4] = '\0';
			int16_t tempint = atoi(num_array);
			if (tempint > max_val_log) {
				max_val_log = tempint;
			}
		}
		entry.close();
	}
	max_val_log++;
	sprintf(LOGFILENAME, "LOG%04d.CSV", max_val_log);
	Serial.print("new file is = ");
	Serial.println(LOGFILENAME);
	
	logFile = SD.open(LOGFILENAME, FILE_WRITE);
	logFile.close();
	
	return;
}

void save_data() {
	/*
	 * This function saves the data in our buffer struct to the SD card.  A timestamp is collected
	 * from the RTC (10 digits), and the data from the buffer are saved directly as bytes, rather
	 * than being converted.  This ensures we have regular length data lines, but requires a
	 * post-processing step to extract human readable values.
	 * 
	 * return: nothing
	 */
	
	// Check that we are not too large of a file to max out the FAT
	if (CNT_LOG_LINES == MAX_LOG_LINES) {
		print_sd_dir();
		CNT_LOG_LINES = 0;
	}
	
	// Open file and save
	logFile = SD.open(LOGFILENAME, FILE_WRITE);
	if (logFile) {
		DateTime now = rtc.now();
		logFile.print(now.unixtime());
		logFile.print(",");
		logFile.print(0x30 + WHICH_UNIT);
		for (size_t i = 2; i < (MSG_SIZE - 4); i++) {
			logFile.print(",");
			logFile.print(cmddata.msg[i], HEX);
		}
		logFile.println();
		logFile.close();
	} else {
		Serial.println("Error loading logfile");
	}
	CNT_LOG_LINES++;
	
	return;
}

void setup() {
	Serial.begin(9600);
	Serial.print("Debugging: ");
	Serial.println(__FILE__);
	
	// Init the software serial used for the RS485 protocol.  Requires the transmit enable.
	Serial1.begin(9600);
	Serial1.flush();
	pinMode(RS485_TE, OUTPUT);
	digitalWrite(RS485_TE, LOW);
	
	// Init RTC
	if (! rtc.begin()) {
		Serial.println("Couldn't find RTC");
		Serial.flush();
		while (1);
	}
	if (! rtc.initialized() || rtc.lostPower()) {
		Serial.println("RTC is NOT initialized, let's set the time!");
		rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
	}
	rtc.start();
	
	// Init SD	
	if (!SD.begin(10)) {
		Serial.println("initialization failed!");
		while (1);
	}
	Serial.println("initialization done.");
	
	// Create file to store data
	print_sd_dir();
	
	// Housekeeping - prepare our unit cycle and store time time
	WHICH_UNIT = MIN_UNIT;
	MIL_CNT = millis();
}

void loop() {
	time_check();
	prep_cmd();
	unit_looper();
	proc_cmd();
	
	cmd_listener();
	cmd_sender();
	
	
}
