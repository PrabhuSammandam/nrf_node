/*
 * delay_cycle.h
 *
 * Created: 3/30/2016 1:15:22 PM
 *  Author: psammand
 */ 


#ifndef DELAY_CYCLE_H_
#define DELAY_CYCLE_H_

#ifndef F_CPU
#warning "F_CPU not defined"
#endif

__attribute__((optimize(3))) static inline void __portable_avr_delay_cycles(unsigned long n)
{
	do { asm volatile("" ::: "memory"); } while (--n);
}

#if !defined(__DELAY_CYCLE_INTRINSICS__)
#define delay_cycles            __portable_avr_delay_cycles
#define cpu_ms_2_cy(ms, f_cpu)  (((uint64_t)(ms) * (f_cpu) + 999) / 6e3)
#define cpu_us_2_cy(us, f_cpu)  (((uint64_t)(us) * (f_cpu) + 999999ul) / 6e6)
#else
#define delay_cycles            __builtin_avr_delay_cycles
#define cpu_ms_2_cy(ms, f_cpu)  (((uint64_t)(ms) * (f_cpu) + 999) / 1e3)
#define cpu_us_2_cy(us, f_cpu)  (((uint64_t)(us) * (f_cpu) + 999999ul) / 1e6)
#endif

#define cpu_delay_ms(delay, f_cpu) delay_cycles(cpu_ms_2_cy(delay, f_cpu))
#define cpu_delay_us(delay, f_cpu) delay_cycles(cpu_us_2_cy(delay, f_cpu))

#define delay_s(delay)      cpu_delay_ms(1000 * delay, F_CPU)
#define delay_ms(delay)     cpu_delay_ms(delay, F_CPU)
#define delay_us(delay)     cpu_delay_us(delay, F_CPU)

#endif /* DELAY_CYCLE_H_ */