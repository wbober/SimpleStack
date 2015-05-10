#ifndef _RSSI_H_
#define _RSSI_H_

void rssi_init();
void rssi_irq_handler();
void rssi_measure();
int16_t rssi_get_value();

#endif
