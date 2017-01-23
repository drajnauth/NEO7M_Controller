#ifndef _MAIN_H_
#define _MAIN_H_

#define LOCATIONS 4
#define EEPROMAddress 0
#define MAX_CHAR 80
#define DEBOUNCE 100

#define LEDSIGNAL 13
#define PBUTTON 10
#define TIMER_INTERVAL 250
#define RECIEVE_TIMTOUT 300

// States 
#define PASSTHROUGH 0x01
#define WAIT4RESP 0x02
#define SENDPOLL 0x04
#define SETFREQ 0x08
#define RESPWAITING 0x10
#define MONITOR 0x20
#define ACK 0x40
#define NAK 0x80
#define INITIALIZE 0x8000

#define CFG 0x6
#define TP5 0x31

struct EEPROM_def {
  unsigned char def;
  long int nLockFreq[LOCATIONS];
  long int LockFreq[LOCATIONS];
};

struct CFGTP5Strut {
  unsigned char Syn1 = 0xB5;
  unsigned char Syn2 = 0x62;
  unsigned char Class = 0x06;
  unsigned char Id = 0x31;
  union {
    unsigned char Lengthb[2];
    unsigned int Length = 0x20;
  } Len;
  unsigned char tpIdx = 0;
  unsigned char reserved0 = 0x1;
  unsigned int reserved1 = 0;
  int antCableDelay = 0x32;
  int rfGroupDelay = 0;
  unsigned long freqPeriod = 1000000;
  unsigned long FreqPeriodLock = 10000000;
  unsigned long pulseLenRatio = 0x80000000;
  unsigned long pulseLenRatioLock = 0x80000000;
  long userConfigDelay = 0;
  unsigned long flags = 0xEF;
  unsigned char chksum1 = 0xB8;
  unsigned char chksum2 = 0x28;
} ;

struct CFGTP5PollStrut {
  unsigned char Syn1 = 0xB5;
  unsigned char Syn2 = 0x62;
  unsigned char Class = 0x06;
  unsigned char Id = 0x31;
  union {
    unsigned char Lengthb[2];
    unsigned int Length = 0x20;
  } Len;
  unsigned char data = 0x00;
  unsigned char chksum1 = 0x38;
  unsigned char chksum2 = 0xE5;
} ;

union gUnion {
  byte b[2];
  unsigned int len;
};


void Reset (unsigned char mode);
void ZeroValues (void);

void ExecuteSerial (char *str);
void EEPROMWrite(int addr);
void EEPROMRead(int addr);

void RecvGPSData ( unsigned char maxpktsize );
void RecvGPSSentence ( void );
void SendRequest (unsigned char gClass, unsigned char gId, unsigned char gPoll );
void DecodePacket (void);

void FlushBuff (void);
void GetCheckSums (void);
void pButton (void);
void TimerInt (void);


#endif // _MAIN_H_
