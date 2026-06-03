#pragma once

#include <PubSubClient.h>

#include "chicken_coop/web.h"

namespace chicken_coop
{
    extern PubSubClient client;

    void taskReadBME280(void *pvParameters);
    void taskAmoniaSensor(void *pvParameters);
}
