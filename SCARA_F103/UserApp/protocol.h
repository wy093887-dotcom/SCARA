#ifndef PROTOCOL_H
#define PROTOCOL_H

void Protocol_Init(void);
void Protocol_Loop(void);
void Protocol_ProcessLine(const char *line);
void Protocol_SendStatus(void);
void Protocol_Tick1kHz(void);

#endif
