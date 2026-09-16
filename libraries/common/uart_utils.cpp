
#include "uart_utils.h"
#include <Arduino.h>

namespace common
{

    void connectToUartWithWait()
    {
        Serial.begin(9600);
        while (!Serial && millis() < 3000)
        {
        }
    }
}