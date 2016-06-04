/*
 * interrupt.h
 *
 * Created: 3/30/2016 1:27:40 PM
 *  Author: psammand
 */ 


#ifndef INTERRUPT_H_
#define INTERRUPT_H_

#include <compiler.h>
//#include <parts.h>

/**
 * \weakgroup interrupt_group
 *
 * @{
 */

#ifdef ISR_CUSTOM_H
#  include ISR_CUSTOM_H
#else

/**
 * \def ISR
 * \brief Define service routine for specified interrupt vector
 *
 * Usage:
 * \code
	ISR(FOO_vect)
	{
	    ...
	}
\endcode
 *
 * \param vect Interrupt vector name as found in the device header files.
 */
#  include <avr/interrupt.h>
#endif // ISR_CUSTOM_H

#  define cpu_irq_enable()     sei()
#  define cpu_irq_disable()    cli()

typedef uint8_t irqflags_t;

static inline irqflags_t cpu_irq_save(void)
{
	irqflags_t flags = SREG;
	cpu_irq_disable();
	return flags;
}

static inline void cpu_irq_restore(irqflags_t flags)
{
	barrier();
	SREG = flags;
}

static inline bool cpu_irq_is_enabled_flags(irqflags_t flags)
{
	return flags & (1 << SREG_I);
}

#define cpu_irq_is_enabled()             cpu_irq_is_enabled_flags(SREG)

//! @}

#endif /* INTERRUPT_H_ */