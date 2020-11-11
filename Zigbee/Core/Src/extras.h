/*
 * extras.h
 *
 *  Created on: Oct 7, 2020
 *      Author: Humphrey Chiramba
 */

#ifndef SRC_EXTRAS_H_
#define SRC_EXTRAS_H_

#include "lib_mrf24j.h"

void handle_rx(mrf_rx_info_t *rxinfo, uint8_t *rx_buffer);
void handle_tx(mrf_tx_info_t *txinfo);

#endif /* SRC_EXTRAS_H_ */
