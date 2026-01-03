#ifndef UART3_H
#define UART3_H

#include "definitions.h"

bool UART3_ReadChar(char *c);
bool UART3_ReadLine(char *buf, size_t maxLen);

void UART3_SendString(const char *str);
void UART3_SendInt(int value);
void UART3_SendFloat(float value, int decimals);

#endif