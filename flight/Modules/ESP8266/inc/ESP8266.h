/*
 * ESP8266.h
 *
 *  Created on: 11 lut 2015
 *      Author: Bobasek
 */

#ifndef ESP8266_H_
#define ESP8266_H_


#define AT "AT\r\n"
//#define ATCWJAP  "AT+CWJAP=\"TP-LINK\",\"zse45rdx\"\r\n"
#define ATCWJAP  "AT+CWJAP=\"quadzik\",\"12345678\"\r\n"
#define ATCIFSR  "AT+CIFSR\r\n"
#define ATCWMODE "AT+CWMODE=3\r\n"
#define ATCIPMUX "AT+CIPMUX=0\r\n"
#define ATCIPCLOSE "AT+CLOSE\r\n"
#define ATCIPSTART "AT+CIPSTART=\"UDP\",\"192.168.1.1\",3333,3333,0\r\n"
#define ATCIPSTATUS "AT+CIPSTATUS\r\n"




int32_t Esp8266Initialize(void);

void initializeEsp8266pin();
void blockingReceiveStringFromEsp8266();
void nonBlockingReceiveStringFromEsp8266();
uint8_t getResponseFromEsp8266();
void retrieveDataFromEsp8266();
uint8_t retrieveDataFromEsp82661();
void processRcvdData();
void processTimeout();
uint8_t confirmData();
uint8_t findColon(uint8_t);
uint8_t configureEsp8266( const char* command,uint16_t timeout);
void createCipStart();
uint8_t changeEsp8266BaudRate();
uint8_t isEsp8266Available();
#endif /* ESP8266_H_ */
