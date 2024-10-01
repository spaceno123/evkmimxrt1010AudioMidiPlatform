/*
 * AudioTask.h
 *
 *  Created on: 2024/09/26
 *      Author: M.Akino
 */

#ifndef AUDIOTASK_H_
#define AUDIOTASK_H_

void AudioTask(void *handle);
void AudioTaskWup(uint32_t bitPattern);
void AudioTask_txComplete(uint8_t *txBuffer);
void AudioTask_rxComplete(uint8_t *rxBuffer);

#endif /* AUDIOTASK_H_ */
