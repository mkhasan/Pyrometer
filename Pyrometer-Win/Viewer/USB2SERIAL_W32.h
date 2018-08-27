#pragma once

#ifndef USBSERIAL_W32_H_
#define USBSERIAL_W32_H_

#define CALLTYPE __stdcall 

#define STX 'X'
#define ETX 'Y'


#define KWSA_ASSERT ASSERT

typedef unsigned char uint8_t;

typedef unsigned int uint32_t;

#define POLYNOM4 0x19
#define POLYNOM8 0x1a9

namespace MY_LIB {
	uint8_t crcCalc(uint32_t const message, int const msgLen, uint32_t polynom);
	int GetDegree(uint32_t polynom);
};


#define INVALID_HANDLE 1
#define ERROR_WRONG_SETTING 2
#define WRITE_ERROR 3
#define WAIT_EVENT_ERROR 4

#define	BAUD_NO	6	//Anzahl moeglicher Baudraten


#define SENSOR_OK 0
#define SENSOR_ERROR -1
#define SENSOR_TRUE 1

#define	SLEEP_IN_THREAD	1
#define DATA_KEY 0xA5
#define	DATA_POSTFIX_1	0x0D
#define	DATA_POSTFIX_2	0x0A
#define PARAM_KEY 0x3B

#define	FRAME_SIZE 10
#define PURGE_LOOP_MAX 15	/* Schleifenzaehler f. PurgeComm-Loop, Zeit: *TIMEOUT_PURGE ms */

#define MUTEX_MAX_WAITTIME 20000//Wartezeit fuer WaitSingleObject(io_mutex)
#define TIMEOUT_PURGE 100	/* Wartezeit beim Abbruch, bis erneuter Abbruch versucht wird */
#define WRITE_TIMEOUT_MULTIPLIER	2	//min Bus-Speed: 9600Bd, 10 Bits/Byte: 10000/9600 <2 ms pro byte
#define WRITE_TIMEOUT_CONSTANT	100	//in ms, wichtig wg.´Schnittstellen-failure, dh ploetzlicher Komm.-Abbruch



#define OWN_ERR_MASK 0x20000000 //Bit29=1, s.http://msdn.microsoft.com/en-us/library/ms680347(VS.85).aspx
#define ERR_MUTEXFAILED 0x300000F0	//d805306608
#define ERR_EVENTFAILED 0x300000F1	//d805306609
#define ERR_PAR_NO 0x300000F2		//d805306610
#define ERR_MEM_ALLOC	0x300000F3	//d805306611 eventuell zuwenig RAM
#define ERR_NO_GSV4_FOUND 0x300000F4	//d805306612 Comport konnte zwar geoeffnet werden, aber kein GSV4 antwortete
#define ERR_BYTES_WRITTEN 0x300000F5	//d805306613
#define ERR_WRONG_PARAMETER 0x30000100	//d805306624
#define ERR_NO_GSV_ANSWER 0x30000058	//d805306456
#define ERR_WRONG_ANSWER_NUM 0x30000059	//d805306457
#define ERR_WRONG_ANSWER 0x30000060	//d805306458
#define ERR_WRONG_FRAME_SUFFIX	0x30000061
#define ERR_NOT_SUPPORTED 0x30000062	//Firmware-Version zu alt
#define ERR_WRONG_COMNO 0x30000101	//d805306625
#define ERR_COM_ALREADY_OPEN	0x300000F6	//d805306614
#define ERR_COM_GEN_FAILURE 0x3000001F //hardware-or driver-error of COMport. See: ERROR_GEN_FAILURE @msdn.microsoft.com/en-us/library/ms681382(VS.85).aspx

#define IN_BUFSIZE	32768 //Windows-buffer incoming
#define OUT_BUFSIZE 256	//Windows-buffer outgoing
#define IN_OWNBUFSIZE	256	//Ablage fuer empfangene Bytes
#define PARAM_INBUF_SIZE 128 //fuer Parameterempfang


#define ACTEX_FLAG_HANDSHAKE	4	//enables HW-handshake in in GSV4activateExtended
#define ACTEX_FLAG_WAIT_EXTENDED	0x0100	//waits longer for device-answer in GSV4activateExtended
#define ACTEX_FLAG_STOP_TX	0x0200	//stops continious data transmission in GSV4activateExtended


#define RESPONSEFRAME_FIXEDSIZE 10	//Parameterantwortframe
#define PARAMRESPONSE_TIMEOUT 6000 //((10000/9600) * 50 * RESPONSEFRAME_FIXEDSIZE) // in ms
#define TIMEOUT_PURGE 100	/* Wartezeit beim Abbruch, bis erneuter Abbruch versucht wird */
#define PURGE_LOOP_MAX 15	/* Schleifenzaehler f. PurgeComm-Loop, Zeit: *TIMEOUT_PURGE ms */
#define STACK_SIZE 65536   /* Stapelgröße für "Hintergrund"-Thread */
#define MUTEX_MAX_WAITTIME 20000//Wartezeit fuer WaitSingleObject(io_mutex)
//#define RELEASE_WAIT_TIME 5000	//Wartezeit fuer GSV4release
#define WAIT_AFTER_STOPTX	200	//Wartezeit nach StopTx-Kommando innerhalb der Befehlsausfuehrung
#define MAX_BUS_LOAD 0.96	//maximale zeitliche Auslastung d. Schnittstellenbusses W: 0.1..1
#define WAIT_ACTIVATE 500 //Wartezeit am Ende der GSV4actExt in ms
#define WAIT_ACTIVATE_FACT 4	//mit ACTEX_FLAG_WAIT_EXTENDED: Wartezeit= WAIT_ACTIVATE*WAIT_ACTIVATE_FACT
#define TRIALS_GET_CMD 5	////mit ACTEX_FLAG_WAIT_EXTENDED: Anz. d. getMode-versuche in GSV4actExt
#define TIMEOUT_GSV_INIT 3000U   /* Erweitertes Timeout zB bei Handshake */
#define TIMEOUT_POLLFLOW 100U   /* Intervall für Abfrage der Hardware-Flow-Control */
#define DELAY_BETWEEN_COMMANDS	80	//in ms, bei Schleifenaufrufen




#define ABORT_ACTIVATE \
	Release; \
   return -1


#define CHECK_COMNO \
	 if (sensor.ComNr != ComNo || sensor.ownPtr==NULL) \
     {	\
		sensor.LastError= ERR_WRONG_COMNO;	\
		return SENSOR_ERROR;	\
	 }

#define START_TX \
	if(sensor.txOn) \
	{			\
		if(StartTX(ComNo)==SENSOR_ERROR) \
			return SENSOR_ERROR; \
	}


#define RES_GETLASTERROR \
	{	\
		sensor.LastError= GetLastError() | OWN_ERR_MASK; \
		res= SENSOR_ERROR; \
	}
#define RES_CHK_BYTES_WRITTEN(no) \
	else if(Byteswritten != (no))	\
	{	\
		sensor.LastError= ERR_BYTES_WRITTEN; \
		res= SENSOR_ERROR;	\
	}


#define RES_MUTEXFAILED	\
	{	\
		sensor.LastError= ERR_MUTEXFAILED; \
		res= SENSOR_ERROR; \
	}

#define WRITE_BYTES_CHK_SUCC(num)	\
	if(!WriteFile(sensor.hcom, txbuf, (num), &Byteswritten, NULL)) \
	{	\
		sensor.LastError= GetLastError() | OWN_ERR_MASK; \
		res= SENSOR_ERROR; \
	}	\
	else if(Byteswritten != (num))	\
	{	\
		sensor.LastError= ERR_BYTES_WRITTEN; \
		res= SENSOR_ERROR;	\
	}


typedef struct _sensordata
{
	volatile unsigned int count;
	volatile unsigned char sender;
	volatile unsigned long value1;
	volatile unsigned long value2;

} sensordata;

typedef struct _sensorRecord	//fuer je 1 GSV-4 mit CHAN_NO=4 Kanaelen
{
	volatile void *ownPtr;
	HANDLE hcom;	//Schnittstelle
	COMMTIMEOUTS timeouts;
	long ComNr;
	long Bitrate;
	unsigned char *in_buffer;   /* Lesepuffer (Zeiger) */
								//unsigned long in_size;
	unsigned long in_pos;   /* Lesepuffer aktuelle Lese-Position */
	unsigned long in_count;   /* Lesepuffer Anzahl Bytes */
	unsigned long in_missing;   /* Lesepuffer Anzahl fehlende Bytes, */
								/* um einen Datenrecord oder Parameterrecord vervollständigen zu können */
	HANDLE thread; //HANDLE thread;
	HANDLE thread_h;
	int terminate;	//=1: Thread beenden
	int is_terminated;
	HANDLE	io_mutex;	//f. Messwerte lesen (Zugriff auf out_buffer)
	HANDLE Prd_mutex;	//f. Parameter lesen (Zugriff auf Param_in_buf)
	HANDLE Prd_event;	//Signal: Alle (Lese-) Parameter fertig empfangen
	HANDLE Clr_event;	//Signal: in_buffer geloescht
						//gsvdata LastValue[CHAN_NO];
	sensordata *out_buffer;   /* Lesepuffer (Zeiger) */
	unsigned long out_size;
	volatile unsigned long out_put;   /* Datenpuffer Einfügeposition */
	volatile unsigned long out_get;   /* Datenpuffer Entnahmeposition */
	volatile unsigned long out_count;   /* Datenpuffer Datensatzanzahl */
	volatile unsigned char* Param_in_buf;	//Ablage f. Read-Parameter
											//volatile unsigned long Param_in_pos;
	volatile unsigned long Param_in_count;
	volatile unsigned long Param_in_missing;
	volatile DWORD error;   /* Ablage für Fehler-Code, write-Zugriff NUR im Lesethread*/
	BOOL clear;	//=1: Buffer loeschen
	BOOL txOn; //=1: Staendige Messwertuebertragung des GSV ist an
	unsigned long FWversion;	//MERGSV$ v.1.1.: Firmware-Version speichern (wenn kommuniziert), sonst =0
	volatile unsigned long LastError;	//jetzt Member, write-Zugriff in Fkt, NICHT im Thread
} sensorRecord;

static void move_data(sensorRecord *tmp);






DWORD WINAPI USB2SERIAL_W32(LPVOID lpParam);
int CALLTYPE SensorStopTX(int ComNo);
int StopTX(int ComNo);
int CALLTYPE SensorGetMode(int ComNo);
int CALLTYPE SensorGetTxMode(int ComNo);
double CalcData(const unsigned long val);
float CalcTemp(unsigned long value1, unsigned long value2);
int CALLTYPE SensorGetValue(int ComNo);
//int CALLTYPE SensorRead(int ComNo, unsigned int * id, double* val1, double* val2);
int CALLTYPE SensorRead(int ComNo, unsigned int* count, unsigned int* addr, double* out1, double* out2);
int CALLTYPE SensorActivate(int ComNo, long Bitrate, long BufSize, long flags);
void Release();


#endif
