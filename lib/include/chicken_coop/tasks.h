#pragma once

#include <PubSubClient.h>

#include "chicken_coop/web.h"

extern PubSubClient client;

void taskMQTT(void *pvParameters); // Spin all the time and keep receiving the messages!
void taskReadBME280(void *pvParameters);
void taskAmoniaSensor(void *pvParameters); // TODO implement amonia sensor

void taskStackMonitor(void *pvParameters); // debug stack monitor for memory usage
// void taskRelay(void *pvParameters);
void taskTcpServer(void *pvParameters); // direct connection