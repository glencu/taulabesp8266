/*
 * ESP8266.c
 *
 *  Created on: 11 lut 2015
 *      Author: Bobasek
 */

#include "openpilot.h"
#include "ESP8266.h"

#include "modulesettings.h"
#include "pios_thread.h"
#include "pios_esp8266.h"
#include "gcsreceiver.h"
#include <inttypes.h>
// ****************
// Private functions

static void Esp8266Task(void *parameters);
static void updateSettings();

// ****************
// Private constants

GCSReceiverData local_gcrdata;
extern GCSReceiverData gcsreceiverdata;

extern void gcsreceiver_updated(UAVObjEvent * ev);

uint8_t ramlog[4000];
uint8_t ramlog_index = 0;
#define TASK_PRIORITY                   PIOS_THREAD_PRIO_HIGHEST
#define STACK_SIZE_BYTES            1850
#define ESP8266_RX_BUFFER_LEN 64
#define ESP8266_COM_TIMEOUT_MS              100
// ****************
// Private variables

uint8_t udpClientOct1 = 0;
uint8_t udpClientOct2 = 0;
uint8_t udpClientOct3 = 0;
uint8_t udpClientOct4 = 0;
char atCipBuffer[100];

static uint32_t Esp8266Port;
static bool module_enabled = false;

static struct pios_thread *esp8266TaskHandle;

static char* esp8266_rx_buffer;

// ****************
/**
 * Initialise the gps module
 * \return -1 if initialisation failed
 * \return 0 on success
 */

int32_t Esp8266Start(void) {
	if (Esp8266Port) {
		// Start gps task
		esp8266TaskHandle = PIOS_Thread_Create(Esp8266Task, "ESP8266",
				STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
		TaskMonitorAdd(TASKINFO_RUNNING_ESP8266, esp8266TaskHandle);
		return 0;
	}

	AlarmsSet(SYSTEMALARMS_ALARM_ESP8266, SYSTEMALARMS_ALARM_CRITICAL);
	return -1;
}

/**
 * Initialise the gps module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
int32_t Esp8266Initialize(void) {
	Esp8266Port = PIOS_COM_ESP8266;

#ifdef MODULE_GPS_BUILTIN
	module_enabled = true;
#else
	uint8_t module_state[MODULESETTINGS_ADMINSTATE_NUMELEM];
	ModuleSettingsAdminStateGet(module_state);
	if (module_state[MODULESETTINGS_ADMINSTATE_ESP8266]
			== MODULESETTINGS_ADMINSTATE_ENABLED) {
		module_enabled = true;
	} else {
		module_enabled = false;
	}
#endif
	memset(ramlog, 0, 4000);

	esp8266_rx_buffer = PIOS_malloc(ESP8266_RX_BUFFER_LEN);
		PIOS_Assert(esp8266_rx_buffer);
		memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_LEN);
	//TODO: updateSettings needs to be fixed
	if (Esp8266Port && module_enabled) {

		PIOS_ESP8266_ResetPin_Init();
		PIOS_ESP8266_Reset();
		updateSettings();

	}
	//PIOS_COM_ChangeBaud(Esp8266Port, 9600);
	AlarmsClear(SYSTEMALARMS_ALARM_ESP8266);

	return 0;

}

MODULE_INITCALL( Esp8266Initialize, Esp8266Start);

uint8_t configureEsp8266(const char* command, uint16_t timeout) {
	PIOS_Thread_Sleep(300);
	PIOS_COM_SendString(Esp8266Port, command);
	PIOS_Thread_Sleep(timeout);
	if (getResponseFromEsp8266() > 3) {
		return 1;
	}
	return 0;
}

void createCipStart() {
	ModuleSettingsESP8266ClientIpOctet1Get(&udpClientOct1);
	ModuleSettingsESP8266ClientIpOctet2Get(&udpClientOct2);
	ModuleSettingsESP8266ClientIpOctet3Get(&udpClientOct3);
	ModuleSettingsESP8266ClientIpOctet4Get(&udpClientOct4);
	memset(atCipBuffer, 0, 100);
	snprintf(atCipBuffer, 100,
			"AT+CIPSTART=\"UDP\",\"%d.%d.%d.%d\",3333,3333,2\r\n",
			udpClientOct1, udpClientOct2, udpClientOct3, udpClientOct4);

}

static void Esp8266Task(void *parameters) {

	uint8_t cntr = 0;
	uint8_t data_changed = 0;
	uint8_t *oldGcr = NULL;
	uint8_t *newGcr = NULL;
	volatile uint32_t numOfBadFrames = 0 ;
	volatile uint32_t timeLast=0;
	volatile uint32_t timeLastMean=0;
	volatile uint32_t shortestTime=0xFFFFFFFF;

	volatile uint8_t packetReceived=0;
	volatile uint32_t numOfFrames = 0 ;
	volatile uint32_t meanTime=0;
	volatile uint32_t sumOfTime=0;

	while (1) {
		PIOS_Thread_Sleep(1000);
		nonBlockingReceiveStringFromEsp8266();
		PIOS_Thread_Sleep(1000);
		nonBlockingReceiveStringFromEsp8266();

		createCipStart();

		PIOS_Thread_Sleep(10);
		configureEsp8266(AT, 700);
		//configureEsp8266(ATCIPCLOSE, 700);
		configureEsp8266(ATCWJAP, 2000);
		//configureEsp8266(ATCIFSR,4000);
		configureEsp8266(ATCWMODE, 700);
		configureEsp8266(ATCIPMUX, 700);
		PIOS_Thread_Sleep(10);
		configureEsp8266(atCipBuffer, 1000);
		PIOS_Thread_Sleep(10);
		//configureEsp8266(ATCIPSTATUS,700);
		while (1) {
			retrieveDataFromEsp8266();

			uint8_t startingIndex = confirmData();

			if (startingIndex != 0) {
				uint8_t colonIndex = findColon(startingIndex);
				if (colonIndex == 0 ) continue;
				data_changed = 0;
				oldGcr = (uint8_t*) &local_gcrdata.Channel[0];
				newGcr = (uint8_t*) &esp8266_rx_buffer[colonIndex+1];
				for (cntr = 0; cntr < 16; cntr++) {

					//if (local_gcrdata.Channel[cntr] != esp8266_rx_buffer[10 + cntr])
					if (oldGcr[cntr] != newGcr[cntr]) {
						data_changed = 1;
						break;
					}
				}

				if (data_changed == 1) {
					memcpy(&local_gcrdata.Channel[0], &esp8266_rx_buffer[colonIndex+1],
							8 * sizeof(uint16_t));
					GCSReceiverSet(&local_gcrdata);
					uint32_t timeNow = PIOS_Thread_Systime();
					if (timeNow - timeLast < shortestTime)
						shortestTime =timeNow - timeLast;
					timeLast = timeNow;

					packetReceived = 1;

					if (timeLastMean == 0 ) timeLastMean= PIOS_Thread_Systime();
				}
				//GCSReceiverUpdated();
			}
			else
			{
				numOfBadFrames++;
			}
			if (packetReceived == 1)
			{
				uint32_t timeNow = PIOS_Thread_Systime();
				sumOfTime += (timeNow-timeLastMean);

				numOfFrames++;

                meanTime = sumOfTime/numOfFrames;
                meanTime++;
                meanTime--;
				timeLastMean = timeNow;
				packetReceived = 0;
			}


		}

	}
}

/**
 * Update the ESP8266 settings, called on startup.
 */
static void updateSettings() {
	if (Esp8266Port) {

		// Retrieve settings
		uint8_t speed;
		ModuleSettingsESP8266SpeedGet(&speed);

		//TODO: Fix baud rate change. Sometimes we will have to find out
		//what baud rate is being used by esp8266

		// Set port speed
		switch (speed) {
		case MODULESETTINGS_ESP8266SPEED_9600:
			//changeEsp8266BaudRate(9600);
			PIOS_COM_ChangeBaud(Esp8266Port, 9600);
			break;
		case MODULESETTINGS_ESP8266SPEED_115200:
		//	changeEsp8266BaudRate(115200);
			PIOS_COM_ChangeBaud(Esp8266Port, 115200);
			break;
		}
	}
}

uint8_t getResponseFromEsp8266() {
	volatile uint16_t loop_cntr = 3;
	uint8_t index = 0;
	while (loop_cntr--) {
		blockingReceiveStringFromEsp8266();

		//TODO: Update return magic numbers
		if (esp8266_rx_buffer[0] == 'O' && esp8266_rx_buffer[1] == 'K')
			return 0;
		if (esp8266_rx_buffer[0] == 'A' && esp8266_rx_buffer[1] == 'L'
				&& esp8266_rx_buffer[2] == 'R' && esp8266_rx_buffer[3] == 'E')
			return 1;

		if (esp8266_rx_buffer[0] == 'E' && esp8266_rx_buffer[1] == 'R'
				&& esp8266_rx_buffer[2] == 'R' && esp8266_rx_buffer[3] == 'O')
			return 2;

		if (esp8266_rx_buffer[1] == ',' && esp8266_rx_buffer[2] == 'C'
				&& esp8266_rx_buffer[3] == 'O' && esp8266_rx_buffer[4] == 'N')
			return 3;

		for (index = 0; index < 63; index++) {
			if (esp8266_rx_buffer[index] == 'O'
				&& esp8266_rx_buffer[index + 1] == 'K')
				return 1;

			if (esp8266_rx_buffer[index] == 'A'
				&& esp8266_rx_buffer[index + 1] == 'L'
				&& esp8266_rx_buffer[index + 2] == 'R'
				&& esp8266_rx_buffer[index + 3] == 'E')
				return 2;

			if (esp8266_rx_buffer[index] == 'E'
				&& esp8266_rx_buffer[index + 1] == 'R'
				&& esp8266_rx_buffer[index + 2] == 'R'
				&& esp8266_rx_buffer[index + 3] == 'O')
				return 3;
		}
		PIOS_Thread_Sleep(2);

	}

	return 4;
}

void nonBlockingReceiveStringFromEsp8266(){

	memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_LEN);
    uint8_t loop_counter=2;
	uint8_t c[64];
	volatile uint8_t buffer_index = 0;
	uint8_t rx_index = 0;
	uint8_t rxd = 0;
	while (loop_counter--) {
		rxd = PIOS_COM_ReceiveBuffer(Esp8266Port, &c[0], 64,
				ESP8266_COM_TIMEOUT_MS);
		if (rxd > 0)
		{
			for (rx_index = 0; rx_index < rxd; rx_index++)
			{
				esp8266_rx_buffer[buffer_index++] = c[rx_index];
				ramlog[ramlog_index++] = c[rx_index];
				if (ramlog_index >= 4000)
					ramlog_index=0;
			}
			/*esp8266_rx_buffer[buffer_index++] = c;
			 ramlog[ramlog_index++] = c; */

			if (esp8266_rx_buffer[buffer_index - 1] == '\n'
					&& esp8266_rx_buffer[buffer_index - 2] == '\r')
				break;
			if (buffer_index >= 63)
				break;


		}

	}

}

void retrieveDataFromEsp8266()
{
	memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_LEN);

		uint8_t c[64],rx_index;
		volatile uint8_t buffer_index = 0;
		uint8_t rxd = 0;
		while (1) {
			//memset(&c[0], 0, 64);
			rxd = PIOS_COM_ReceiveBuffer(Esp8266Port, &c[0], 64,
					ESP8266_COM_TIMEOUT_MS);
			if (rxd > 0)
			{
				for (rx_index = 0; rx_index < rxd; rx_index++)
				{
					esp8266_rx_buffer[buffer_index++] = c[rx_index];
				    ramlog[ramlog_index++] = c[rx_index];
				    if (ramlog_index >= 4000)
				    	ramlog_index=0;
				}
				/*esp8266_rx_buffer[buffer_index++] = c;
				 ramlog[ramlog_index++] = c;*/

				if (esp8266_rx_buffer[buffer_index - 1] == '\n'
						&& esp8266_rx_buffer[buffer_index - 2] == '\r')
					break;
				if (buffer_index >= 63)
					break;


			}

		}

}

void blockingReceiveStringFromEsp8266() {

	memset(esp8266_rx_buffer, 0, ESP8266_RX_BUFFER_LEN);

	uint8_t c[64];
	volatile uint8_t buffer_index = 0;
	uint8_t rx_index = 0;
	uint8_t rxd = 0;
	while (1) {
		rxd = PIOS_COM_ReceiveBuffer(Esp8266Port, &c[0], 64,
				ESP8266_COM_TIMEOUT_MS);
		if (rxd > 0)
		{
			for (rx_index = 0; rx_index < rxd; rx_index++)
			{
				esp8266_rx_buffer[buffer_index++] = c[rx_index];
			    ramlog[ramlog_index++] = c[rx_index];
			    if (ramlog_index >= 4000)
			    	ramlog_index=0;
			}
			/*esp8266_rx_buffer[buffer_index++] = c;
			 ramlog[ramlog_index++] = c; */

			if (esp8266_rx_buffer[buffer_index - 1] == '\n'
					&& esp8266_rx_buffer[buffer_index - 2] == '\r')
				break;
			if (buffer_index >= 63)
				break;


		}

	}

}

uint8_t confirmData()
{
   	uint8_t index=0;

   	for(index = 0 ; index < 60 ; index++)
   	{
	if (esp8266_rx_buffer[index] == '+' && esp8266_rx_buffer[index+1] == 'I'
						&& esp8266_rx_buffer[index+2] == 'P'
						&& esp8266_rx_buffer[index+3] == 'D')
		return index;
   	}

   	return 0;

}

uint8_t findColon(uint8_t startingIndex)
{
	uint8_t index=0;

	for(index = startingIndex; index < 64 ; index++)
	{
		if(esp8266_rx_buffer[index] == ':')
			return index ;
	}

	return 0;
}

uint8_t changeEsp8266BaudRate(uint32_t speed)
{
	PIOS_Thread_Sleep(10);
	memset(atCipBuffer, 0, 100);
	snprintf(atCipBuffer, 100,"AT+UART=%u,8,1,0,0\r\n",(unsigned int)speed);
	PIOS_COM_SendString(Esp8266Port, atCipBuffer);

	return 0;
}

uint8_t isEsp8266Available()
{
	if (configureEsp8266(AT, 700) == 0 )
		return 1; // :) yeah , I know :P
	return 0;
}
