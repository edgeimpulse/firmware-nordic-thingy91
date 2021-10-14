#ifndef CONNECTIVITY_H
#define CONNECTIVITY_H
#include <stdbool.h>

void ei_mqtt_publish(const char *value);
int ei_mqtt_connect(const char *client_id);
bool ei_get_mqtt_connected(void);

#endif /* CONNECTIVITY_H */