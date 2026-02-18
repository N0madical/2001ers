#include "utils.h"

void TeleplotPrint(const char* var, float value)
{
    Serial.print('>');
    Serial.print(var);
    Serial.print(':');
    Serial.print(value);
    Serial.print('\n');
}

void TeleplotPrintTuple(const char* var, float value1, float value2)
{
    Serial.print('>');
    Serial.print(var);
    Serial.print(':');
    Serial.print('(');
    Serial.print(value1);
    Serial.print(',');
    Serial.print(value2);
    Serial.print(')');
    Serial.print('\n');
}

