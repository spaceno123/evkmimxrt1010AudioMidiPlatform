/*
 * circure.h
 *
 *  Created on: 2024/09/20
 *      Author: M.Akino
 */

#ifndef CIRCURE_H_
#define CIRCURE_H_

typedef struct circure_ {
	uint16_t wpos;
	uint16_t rpos;
	uint16_t size;
	uint8_t *buf;
} circure_t;

static inline void circure_clear(circure_t *p)
{
	p->wpos = p->rpos = 0;
}

static inline int16_t circure_get(circure_t *p)
{
	int16_t ret = -1;

	if (p->wpos != p->rpos)
	{
		uint16_t rnxt = p->rpos + 1;
		ret = p->buf[p->rpos];
		p->rpos = rnxt >= p->size ? 0 : rnxt;
	}

	return ret;
}

static inline int16_t circure_put(circure_t *p, uint8_t dt)
{
	int16_t ret = -1;
	uint16_t wnxt = p->wpos + 1;
	wnxt = wnxt >= p->size ? 0 : wnxt;
	if (wnxt != p->rpos)
	{
		p->buf[p->wpos] = dt;
		p->wpos = wnxt;
		ret = dt;
	}
}

static inline int16_t circure_remain(circure_t *p)
{
	int16_t ret = p->wpos - p->rpos;

	if (ret < 0)
	{
		ret += p->size;
	}

	return ret;
}

#endif /* CIRCURE_H_ */
