#ifndef _UART_H_
#define _UART_H_


#define RBUFF 80		// Max RS232 Buffer Size
#define MAXIMUM_DIGITS 6	// Max numerical digits to process
#define MAX_COMMAND_ENTRIES 6 

#define QUIET 0x1
#define ECHOONLY 0x2


char ProcessSerial ( void ) ;
unsigned char ParseSerial ( char *str );
void ResetSerial (void);
void ErrorOut ( void );

#endif // _UART_H_




