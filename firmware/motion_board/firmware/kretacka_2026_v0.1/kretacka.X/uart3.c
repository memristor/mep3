#include "uart3.h"

bool UART3_ReadChar(char *c)
{
    if (UART3_ReadIsBusy())
        return false;

    UART3_Read(c, 1);
    return true;
}

bool UART3_ReadLine(char *buf, size_t maxLen)
{
    static size_t idx = 0;
    char c;

    if (!UART3_ReadChar(&c))
        return false;

    if (c == '\n')
    {
        buf[idx] = '\0';
        idx = 0;
        return true;
    }

    if (idx < maxLen - 1)
    {
        buf[idx++] = c;
    }

    return false;
}

void UART3_SendString(const char* str) {
    UART3_Write((uint8_t*)str, strlen(str));
    while (UART3_WriteIsBusy());  // Wait until done sending
}

void UART3_SendInt(int value) {
    char buffer[12];
    sprintf(buffer, "%d", value);
    UART3_SendString(buffer);
}

void UART3_SendFloat(float value, int decimals) {
    char buffer[32];
    char format[8];
    sprintf(format, "%%.%df", decimals);
    sprintf(buffer, format, value);
    UART3_SendString(buffer);
}