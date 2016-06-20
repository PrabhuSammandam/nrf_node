/*
* spi.h
*
* Created: 20-03-2016 19:00:06
*  Author: prabhu
*/


#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>

void spi_init();

__always_inline static inline uint8_t spi_write_byte(uint8_t data)
{
	SPDR = data;
	/*
	* The following NOP introduces a small delay that can prevent the wait
	* loop form iterating when running at the maximum speed. This gives
	* about 10% more speed, even if it seems counter-intuitive. At lower
	* speeds it is unnoticed.
	*/
	asm volatile("nop");

	loop_until_bit_is_set(SPSR,SPIF);

	return SPDR;
}

__always_inline static inline uint16_t spi_write_word(uint16_t data)
{
	union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } in, out;
	in.val = data;

	if (!(SPCR & _BV(DORD))) {
		SPDR = in.msb;
		asm volatile("nop"); // See transfer(uint8_t) function
		loop_until_bit_is_set(SPSR,SPIF);
		out.msb = SPDR;
		SPDR = in.lsb;
		asm volatile("nop");
		loop_until_bit_is_set(SPSR,SPIF);
		out.lsb = SPDR;
	}
	else {
		SPDR = in.lsb;
		asm volatile("nop");
		loop_until_bit_is_set(SPSR,SPIF);
		out.lsb = SPDR;
		SPDR = in.msb;
		asm volatile("nop");
		loop_until_bit_is_set(SPSR,SPIF);
		out.msb = SPDR;
	}
	return out.val;
}

__always_inline static inline void spi_write_buffer(void* buf, uint16_t size)
{
	if (size == 0) return;

	uint8_t *p = (uint8_t *)buf;
	SPDR = *p;
	while (--size > 0) {
		uint8_t out = *(p + 1);
		loop_until_bit_is_set(SPSR,SPIF);
		uint8_t in = SPDR;
		SPDR = out;
		*p++ = in;
	}
	loop_until_bit_is_set(SPSR,SPIF);
	*p = SPDR;
}

#endif /* SPI_H_ */