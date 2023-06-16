#ifndef AVR_BITOPS_H
#define AVR_BITOPS_H

/**
	File: bitops.h
	Generic bit and bit field manipulation macros.
*/

/**
	Macro: BV
	Utility macro that maps a bit number to an integer value with (only)
	the corresponding bit set to 1. Function-like macro.
	
	Parameters:
		N - A bit number. Should be in the range 0-7 for 8-bit values
			and 0-15 for 16-bit values. Bit 0 is the least significant
			and bit 7/15 the most significant.
	
	Returns:
		An integer value with (only) the specified bit set to one (1).
*/
#define BV(N) (1 << (N))

#define BITMASK(N) (BV(N) - 1)

#define BITMASK_AT(N, O) (BITMASK(N) << (O))

#define BITMASK_NOT_AT(N, O) (~BITMASK_AT(N, O))

#define BITSTATES(N) BV(N)

#define BITMAX(N) BITMASK(N)

#define GET_BITFIELD(S, D) (BITMASK(S) & (D))

#define GET_BITFIELD_AT(S, O, D) GET_BITFIELD(S, (D) >> (O))

#define MAKE_BITFIELD_AT(S, O, X) (GET_BITFIELD(S, X) << (O))

#define CLEAR_BITFIELD_AT(S, O, D) (BITMASK_NOT_AT(S, O) & (D))

#define SET_BITFIELD_AT(S, O, D, X) (CLEAR_BITFIELD_AT(S, O, D) | MAKE_BITFIELD_AT(S, O, X))

#endif
