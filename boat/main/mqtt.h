#pragma once

#define BOATIN "boat/in"
#define BOATTEMP "boat/temp"
#define BOATPRES "boat/pres"
#define BOATHUM "boat/hum"
#define BOATFILE "boat/file"

void mqtt_init();
char *get_topic_data();
void mqtt_publish(char *topic, char *data);