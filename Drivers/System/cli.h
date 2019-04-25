/*

*/

///////////////////////////////////////////////////////////////////////////////

#pragma once

#define CLI_RATE		(250000)

///////////////////////////////////////////////////////////////////////////////

extern uint8_t (*cliPortAvailable)(void);
extern void     (*cliPortClearBuffer)(void);
extern uint8_t  (*cliPortRead)(void);
extern void     (*cliPortPrint)(char *str);
extern void     (*cliPortPrintCh)(uint8_t ch);
extern void     (*cliPortPrintF)(const char * fmt, ...);
extern void     (*cliPortPrintBinary)(uint8_t *buf, uint16_t length);

///////////////////////////////////////

extern uint8_t cliBusy;

extern uint32_t sonar_altitude;

extern float debug1, debug2;

///////////////////////////////////////////////////////////////////////////////
// Read Float from CLI
///////////////////////////////////////////////////////////////////////////////

float readFloatCLI(void);

///////////////////////////////////////////////////////////////////////////////
// Read Character String from CLI
///////////////////////////////////////////////////////////////////////////////

char *readStringCLI(char *data, uint8_t length);


///////////////////////////////////////////////////////////////////////////////

void cliCom(void);
int CliInit (void);