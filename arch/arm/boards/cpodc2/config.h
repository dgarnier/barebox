#ifndef __CONFIG_H
#define __CONFIG_H

#include <mach/cpu.h>
/* set clock based on what chip id we have...
 * ... it would be better to either set it exclusively
 * ... or get rid of this variable and have it computed by
 * ... the cpu
 * assuming no bootstrap
 */
#define AT91_MAIN_CLOCK	 (cpu_is_at91sam9g10() ? 20000000 : 18432000)

#endif	/* __CONFIG_H */
