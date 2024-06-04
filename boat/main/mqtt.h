#pragma once

#define BOATIN "boat/in"
#define BOATOUT "boat/out"
#define BOATFILE "boat/file"

void mqtt_init();
char *get_topic_data();
void mqtt_publish(char *topic, char *data);