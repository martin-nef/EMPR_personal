/***********************************************************************/
/* Type group ----------------------------------------------------------- */
#ifndef LPC_TYPES_H
#define LPC_TYPES_H

/* Includes ------------------------------------------------------------------- */
#include <stdint.h>


/* Public Types --------------------------------------------------------------- */
typedef enum {FALSE = 0, TRUE = !FALSE} Bool;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, IntStatus, SetState;
#define PARAM_SETSTATE(State) ((State==RESET) || (State==SET))

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define PARAM_FUNCTIONALSTATE(State) ((State==DISABLE) || (State==ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} Status;


typedef enum
{
        NONE_BLOCKING = 0,              
        BLOCKING,                               
} TRANSFER_BLOCK_Type;


typedef void (*PFV)();

typedef int32_t(*PFI)();

/* Public Macros -------------------------------------------------------------- */
/* _BIT(n) sets the bit at position "n"
 * _BIT(n) is intended to be used in "OR" and "AND" expressions:
 * e.g., "(_BIT(3) | _BIT(7))".
 */
#undef _BIT
/* Set bit macro */
#define _BIT(n) (1<<n)

/* _SBF(f,v) sets the bit field starting at position "f" to value "v".
 * _SBF(f,v) is intended to be used in "OR" and "AND" expressions:
 * e.g., "((_SBF(5,7) | _SBF(12,0xF)) & 0xFFFF)"
 */
#undef _SBF
/* Set bit field macro */
#define _SBF(f,v) (v<<f)

/* _BITMASK constructs a symbol with 'field_width' least significant
 * bits set.
 * e.g., _BITMASK(5) constructs '0x1F', _BITMASK(16) == 0xFFFF
 * The symbol is intended to be used to limit the bit field width
 * thusly:
 * <a_register> = (any_expression) & _BITMASK(x), where 0 < x <= 32.
 * If "any_expression" results in a value that is larger than can be
 * contained in 'x' bits, the bits above 'x - 1' are masked off.  When
 * used with the _SBF example above, the example would be written:
 * a_reg = ((_SBF(5,7) | _SBF(12,0xF)) & _BITMASK(16))
 * This ensures that the value written to a_reg is no wider than
 * 16 bits, and makes the code easier to read and understand.
 */
#undef _BITMASK
/* Bitmask creation macro */
#define _BITMASK(field_width) ( _BIT(field_width) - 1)

/* NULL pointer */
#ifndef NULL
#define NULL ((void*) 0)
#endif

/* Number of elements in an array */
#define NELEMENTS(array)  (sizeof (array) / sizeof (array[0]))

/* Static data/function define */
#define STATIC static
/* External data/function define */
#define EXTERN extern

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/* Old Type Definition compatibility ------------------------------------------ */
typedef char CHAR;

typedef uint8_t UNS_8;

typedef int8_t INT_8;

typedef uint16_t UNS_16;

typedef int16_t INT_16;

typedef uint32_t UNS_32;

typedef int32_t INT_32;

typedef int64_t INT_64;

typedef uint64_t UNS_64;

typedef Bool BOOL_32;

typedef Bool BOOL_16;

typedef Bool BOOL_8;

#endif /* LPC_TYPES_H */

/* --------------------------------- End Of File ------------------------------ */