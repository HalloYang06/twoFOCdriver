#ifndef H7FOC_COMM_ETH_STACK_LAN9252_PORT_H
#define H7FOC_COMM_ETH_STACK_LAN9252_PORT_H

#include <stdint.h>

typedef void (*lan9252_port_irq_cb_t)(void);

void lan9252_port_open(void);
void lan9252_port_cs_low(void);
void lan9252_port_cs_high(void);
uint8_t lan9252_port_spi_transfer(uint8_t tx_byte);

uint32_t lan9252_port_irq_lock(void);
void lan9252_port_irq_unlock(uint32_t irq_state);

void lan9252_port_set_irq_cb(lan9252_port_irq_cb_t cb);
void lan9252_port_set_sync0_cb(lan9252_port_irq_cb_t cb);
void lan9252_port_set_sync1_cb(lan9252_port_irq_cb_t cb);

uint32_t lan9252_port_get_tick_ms(void);
void lan9252_port_reset_tick_ms(void);

#endif
