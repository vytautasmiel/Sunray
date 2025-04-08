#ifndef MQTT_H
#define MQTT_H

#include "config.h" 

#ifdef USE_MQTT

void mqtt_setup();
void mqtt_loop();

#endif 

#endif 