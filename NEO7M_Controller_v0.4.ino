#include <TimerOne.h>

 
#include <EEPROM.h>
#include <stdint.h>
#include <SoftwareSerial.h>


#include "UART.h"              // VE3OOI Serial Interface Routines (TTY Commands)
#include "NEO7M_Controller.h"


SoftwareSerial mySerial(12, 11); // RX, TX

// Packet buffer and processing

unsigned char Buf [MAX_CHAR];
unsigned char rcv;
unsigned char rstart, sendok;
unsigned char pos, buflen;
unsigned int states;
unsigned int rcvtimeout;

// LED Blinking
unsigned char bCnt, bTrigger, bPause;
unsigned long blinkTime;

// Push Button
unsigned char ButtonState;
unsigned long bTime0, bTime1;


// Structures for packets
union gUnion un;
CFGTP5Strut tp5;
CFGTP5PollStrut tp5poll;
CFGTP5Strut *ptp5;
CFGTP5PollStrut *ptp5poll;

// Frequencies stored in memories
EEPROM_def ee;


// UART Command Line Parameters
extern char rbuff[RBUFF]; 
extern char commands[MAX_COMMAND_ENTRIES];
extern unsigned char command_entries;
extern unsigned long numbers[MAX_COMMAND_ENTRIES];
extern unsigned char ctr;

void setup() 
{   
  pinMode(LEDSIGNAL, OUTPUT);
  pinMode(PBUTTON, INPUT_PULLUP);
  digitalWrite(LEDSIGNAL, HIGH);
  
  Serial.begin (9600);
  Serial.println ("\r\nNEO7M GPS Controller v0.4");
  Serial.write ("RDY> ");
  Serial.flush();
  
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);

  Reset(1);        // Read EEPROM

  delay(2000);    // Wait for GPS to startup

  Reset(2);       // Set Default Frequency and set into passthrough mode

}


void loop() 
{  
  
  if (states & SENDPOLL && sendok) {
    SendRequest(CFG, TP5, 1);
    Reset(0);
    states |= WAIT4RESP;
    states &= ~SENDPOLL;
    RecvGPSData(sizeof(tp5));
    
  } else if (states & SETFREQ && sendok) {
    SendRequest(CFG, TP5, 0);
    Reset(0);
    states |= WAIT4RESP;
    states &= ~SETFREQ;
    RecvGPSData(sizeof(tp5poll));
    
  } else {
    ProcessSerial ();
    RecvGPSSentence();
    BlinkLED ();
    pButton();
    if (ButtonState) {
      ButtonState = 0;
      ee.def++;
      if (ee.def > LOCATIONS) ee.def = 1;
      Serial.print ("Def: ");
      Serial.println (ee.def);
      EEPROMWrite (EEPROMAddress); 
      tp5.freqPeriod = ee.nLockFreq[ee.def-1];
      tp5.FreqPeriodLock = ee.LockFreq[ee.def-1];
      SendRequest(CFG, TP5, 0);
      Reset(0);
      states |= WAIT4RESP;
      RecvGPSData(sizeof(tp5poll));
    } 
  }
  
  if (states & RESPWAITING) {
    DecodePacket();
  }
  
}

void DecodePacket (void)
{
  ptp5 = (CFGTP5Strut *) Buf;
  ptp5poll = (CFGTP5PollStrut *) Buf;

  if (ptp5poll->Class == 0x05 && ptp5poll->Id == 0x01) {
     Serial.println ("\r\nFreq Set");
     states &= ~RESPWAITING;
     states &= ~SENDPOLL;
     states &= ~SETFREQ;
     states |= ACK;
      
   } else if (ptp5poll->Class == 0x05 && ptp5poll->Id == 0x01) {
     Serial.println ("\r\nGPS Err");
     states &= ~RESPWAITING;    // Don't clear SENDPOLL or SETFREA - Resend.
     states |= NAK;
     
   } else if (ptp5poll->Class == 0x06 && ptp5poll->Id == 0x31) {
     Serial.print ("\r\nFreq: ");
     Serial.print (ptp5->freqPeriod);
     Serial.print ("\r\nLCK Freq: ");
     Serial.println (ptp5->FreqPeriodLock);
     states &= ~RESPWAITING;
     states &= ~SENDPOLL;
     states &= ~SETFREQ;
     states |= ACK;
   } else {
     Serial.println("\r\nDecode Err");
     states = 0;
   }
   Reset(0);
  
}

void BlinkLED (void)
{
  if (!blinkTime) {
    blinkTime = millis();
  }
  if (millis()-blinkTime > 250) {
    blinkTime = 0;
    bTime1 = 0;
    TimerInt();
  }
}


void TimerInt (void)
{
  if (bPause > 3) {
    if (bCnt++ < 2*ee.def) {
      if (bTrigger) {
        digitalWrite(LEDSIGNAL, LOW);
        bTrigger = 0;
      } else {
        digitalWrite(LEDSIGNAL, HIGH);
        bTrigger = 1;
      }
    } else { 
      digitalWrite(LEDSIGNAL, LOW);
      bCnt = 0;
      bPause = 0;
    }
  } else {
    bPause++;
  }
  
}  

void pButton ( void )
{
  
  if (digitalRead(PBUTTON)==HIGH) {
    if (!bTime0) {
      bTime1 = 0;
      bTime0 = millis();
    }
    if (millis()-bTime0 > DEBOUNCE) {
      bTime0 = 0;
      bTime1 = 0;
      ButtonState =  0;
    }
  } else {
    if (!bTime1) {
      bTime0 = 0;
      bTime1 = millis();
    }
    if (millis()-bTime1 > DEBOUNCE) {
      bTime0 = 0;
      bTime1 = 0;
      ButtonState =  1;
    }
  }    
}  




void RecvGPSData ( unsigned char maxpktsize )
{
  unsigned char i;
  
  for (i=0; i<maxpktsize; i++) {                  
// At 9600bps, its take 104 us for each character or 833us for a byte, with control characters its about 1146 us
// The internal buffer is 64 bytes and will be filled in 53ms to fill up
/*
    if (mySerial.overflow()) {
      Serial.println ("Serial OVFL");
      while (mySerial.available()) rcv = mySerial.read();
      Reset(1);
      return;
    }
*/
    while (mySerial.available()) {
      rcv = mySerial.read();
      if ( !rstart && rcv == 0xB5) {
        rstart = 1;
        pos = 0;
        buflen = 0;
        Buf[pos++] = rcv;
      } else if (rstart) {
        Buf[pos]=rcv;
        switch (pos) {
          case 1: 
            if (rcv != 0x62) {
              Serial.println ("SYN Err");
              Reset(1);
              return;
            }
            break;
            
          case 4:
            un.b[0] = rcv;
            break;
            
          case 5:
            un.b[1] = rcv;
            if (un.len > maxpktsize) {
              Serial.println ("Len Err");
              Reset(1);
              return;
            }
            break;
            
          default:
            if (pos > 5 && pos < MAX_CHAR) {
              if ( (pos-6) > un.len ) {
                buflen = pos;
                states &= ~WAIT4RESP;
                states |= RESPWAITING;
                rstart = 0;
                return;
              }
            } else if (pos >= MAX_CHAR) {
              Serial.println ("Pkt OVRN");
              Reset(1);
              return;
            }
        } 
        pos++;
      } 
    }
    delayMicroseconds(1146);
  }
}



void RecvGPSSentence ( void )
{
  while (mySerial.available()) {
    rcv = mySerial.read();
    if (states & MONITOR || states & PASSTHROUGH) Serial.write (rcv);
    if (rcv == 0x0A) sendok = 1;
    else sendok = 0;
  }
}


void FlushBuff ( void )
{
  unsigned char i;
  Serial.print ("\r\nBuf: ");
  Serial.println (buflen);
  for (i=0; i<pos; i++) {
    Serial.println (Buf[i], HEX);
  }
}

void SendRequest (unsigned char gClass, unsigned char gId, unsigned char gPoll )
{
  unsigned char i;
  unsigned char *ptr;
  
  if (gClass == CFG && gId == TP5 && gPoll) {
    tp5poll.Syn1 = 0xB5;
    tp5poll.Syn2 = 0x62;
    tp5poll.Class = 0x06;
    tp5poll.Id = 0x31;
    tp5poll.Len.Length = 0x01;
    tp5poll.data = 0x00;
    tp5poll.chksum1 = 0x38;
    tp5poll.chksum2 = 0xE5;

    ptr = (unsigned char *) &tp5poll;
    for (i=0; i<sizeof(tp5poll); i++) {
      mySerial.write ((uint8_t)*ptr);
      ptr++;
    }
    mySerial.flush ();
    
  } else if (gClass == CFG && gId == TP5 && !gPoll) {
    tp5.Syn1 = 0xB5;
    tp5.Syn2 = 0x62;
    tp5.Class = 0x06;
    tp5.Id = 0x31;
    tp5.Len.Length = 0x20;
    tp5.tpIdx = 0;
    tp5.reserved0 = 0x1;
    tp5.reserved1 = 0;
    tp5.antCableDelay = 0x32;
    tp5.rfGroupDelay = 0;
//    tp5.freqPeriod = 1000000;    //0xF4240
//    tp5.FreqPeriodLock = 10000000;  //0x989680
    tp5.pulseLenRatio = 0x80000000;
    tp5.pulseLenRatioLock = 0x80000000;
    tp5.userConfigDelay = 0;
    tp5.flags = 0xEF;
    tp5.chksum1 = 0xB8;
    tp5.chksum2 = 0x28;

    GetCheckSums();

    ptr = (unsigned char *) &tp5;
    for (i=0; i<sizeof(tp5); i++) {
      mySerial.write ((uint8_t)*ptr);
      ptr++;
    }
    mySerial.flush ();
  }
}  

void GetCheckSums ( void )
{
  unsigned char i;
  unsigned char *ptr;
  unsigned int CK_A, CK_B;
  CK_A = 0;
  CK_B = 0;

  ptr = (unsigned char *) &tp5.Class;
  for (i=0; i<(sizeof(tp5)-4); i++) {
    CK_A += (unsigned char)*ptr;
    CK_B += CK_A;
    ptr++;
  }

  tp5.chksum1 = (unsigned char) (CK_A & 0xFF);
  tp5.chksum2 = (unsigned char) (CK_B & 0xFF);
  
}

void ExecuteSerial (char *str)
{
  
// num defined the actual number of entries process from the serial buffer
// i is a generic counter
  unsigned char num;
  unsigned long i;
  
// This function called when serial input in present in the serial buffer
// The serial buffer is parsed and characters and numbers are scraped and entered
// in the commands[] and numbers[] variables.
  num = ParseSerial (str);

// Process the commands
// Note: Whenever a parameter is stated as [CLK] the square brackets are not entered. The square brackets means
// that this is a command line parameter entered after the command.
// E.g. F [CLK] [FREQ] would be mean "F 0 7000000" is entered (no square brackets entered)

  if ( (states & PASSTHROUGH) && commands[0] == 'R' && commands[1] == 'R' && commands[2] == 'R') {
    Reset(1);
    commands[0] = 'R';
    Serial.println ("\r\n\r\nCmd Mode\r\nRDY> ");
    
  } else if (states & PASSTHROUGH) {
    return;
  }
  
  switch (commands[0]) {
   
    case 'D':
      Serial.print ("State: ");
      Serial.println (states, HEX);
      EEPROMRead (EEPROMAddress); 
      for (i=0; i<LOCATIONS; i++) {
        Serial.print ("Loc: ");
        Serial.print (i+1);
        Serial.print (" NotLoc Freq: ");
        Serial.print (ee.nLockFreq[i]); 
        Serial.print (" Loc Freq: ");
        Serial.println (ee.LockFreq[i]);
      }  
      Serial.print ("Default: ");
      Serial.println (ee.def);
      break;
    
    case 'F':             // Set Frequency
      if (numbers[0] > 10000000 || !numbers[0]) {
        Serial.print ("Bad NLock Freq");
        break;
      }  
      
      if (numbers[1] > 10000000 || !numbers[1]) {
        Serial.print ("Bad Lock Freq");
        break;
      }  
      
      tp5.freqPeriod = numbers[0];    //0xF4240
      tp5.FreqPeriodLock = numbers[1];  //0x989680
    
      states |= SETFREQ;
      break;

    // Help Screen. This consumes a ton of memory but necessary for those
    // without much computer or programming experience.
    case 'H':             // Help
      Serial.println ("D - Dump EEprom");
      Serial.println ("F [nHz] [lHz] - Set Freq for NonLck Freq and Lck Freq");
      Serial.println ("\tE.g. F 1000 7102000");
      Serial.println ("M - Monitor NMEA");
      Serial.println ("P - Poll Freq");
      Serial.println ("R - Reset");
      Serial.println ("S D [c] - Set default location");
      Serial.println ("\tE.g. S D 3");
      Serial.println ("S L|N [c] [HZ] - Set Freq Memory c for Lock or NonLock");
      Serial.println ("\tE.g. S L 0 10000000");
      break;
      
    case 'M':
      if (states & MONITOR) {
        states &= ~MONITOR; 
      } else {
        states |= MONITOR; 
      }
      break;
 
 
    case 'P':             // Poll and get Frequency
      states |= SENDPOLL;
      break;

    case 'R':             // Reset
      Reset (1);
      break;

    case 'S':             // Set Frequency
      if (commands[1] == 'D') {
        if (numbers[0] > LOCATIONS || !numbers[0]) {
          Serial.print ("Bad Default Loc");
          break;
        }  
        ee.def = numbers[0];
        EEPROMWrite (EEPROMAddress); 
        Reset(2);
        states = 0;
        break;
      }
    
      if (numbers[0] > LOCATIONS || !numbers[0]) {
        Serial.print ("Bad Mem Loc");
        break;
      }  
      if (numbers[1] > 10000000 || !numbers[1]) {
        Serial.print ("Bad Freq");
        break;
      }  
      if (commands[1] == 'N') ee.nLockFreq[numbers[0]-1] = numbers[1];
      else if (commands[1] == 'L') ee.LockFreq[numbers[0]-1] = numbers[1];
      else {
        Serial.println ("L or N Missing");
        break;
      }  
      
      EEPROMWrite (EEPROMAddress); 
      break;
     
    // If an undefined command is entered, display an error message
    default:
      ErrorOut ();
  }
  
}

void Reset (unsigned char mode)
{
  switch (mode) {
    case 1:
      ZeroValues();
      states = 0;
      EEPROMRead(EEPROMAddress);
      ResetSerial ();
      break;
   
    case 2:
      ZeroValues();
      if (ee.def <= LOCATIONS && ee.def) {
        tp5.freqPeriod = ee.nLockFreq[ee.def-1];
        tp5.FreqPeriodLock = ee.LockFreq[ee.def-1];
      } else {
        tp5.freqPeriod = 1000;    
        tp5.FreqPeriodLock = 1000;
      }
      SendRequest(CFG, TP5, 0);
      ZeroValues();
      states = WAIT4RESP;
      RecvGPSData(sizeof(tp5));
      DecodePacket();
      ZeroValues();
      states = PASSTHROUGH;
      break;
      
    default:
      ZeroValues();
  }

  
}



void ZeroValues (void)
{
  rstart = 0;
  pos = 0;
  buflen = 0;
  sendok = 0;
  rcvtimeout = 0;
  bCnt = 0;
  bTrigger = 0;
  bPause = 0;
  bTime0 = 0; 
  bTime1 = 0;
  blinkTime = 0;
  ButtonState = 0;
  memset ((char *)&Buf, 0, sizeof(Buf));
}  
  
// This routines are NOT part of the Si5351 and should not be included as part of the Si5351 routines.
// Note that some arduino do not have eeprom and would generate an error during compile.
// If you plan to use a Arduino without eeprom then you need to hard code a calibration value.

void EEPROMWrite (int addr)
{
  EEPROM.put (addr, ee); 
}

void EEPROMRead(int addr)
{
  EEPROM.get (addr, ee); 
}



