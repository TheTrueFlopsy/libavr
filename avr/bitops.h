#ifndef AVR_BITOPS_H
#define AVR_BITOPS_H

/**
	File: bitops.h
	Generic bit and bit field manipulation macros.
*/

/**
	Macro: BITOPS_1
	The expression the bitops macros will use to represent an integer
	with the least significant bit set to one (1) and the other bits
	set to zero (0). Configuration macro.
	
	Default value: 1U
*/
#ifndef BITOPS_1
#define BITOPS_1 1U
#endif

/**
	Macro: BV
	Maps a bit number to an integer value with (only)
	the corresponding bit set to 1. Function-like macro.
	
	Parameters:
		N - A bit number. Should be in the range 0-7 for 8-bit values
			and 0-15 for 16-bit values. Bit 0 is the least significant
			and bit 7/15 the most significant.
	
	Returns:
		An integer value with (only) the specified bit set to one (1).
*/
#define BV(N) (BITOPS_1 << (N))

/**
	Macro: BITMASK
	Maps a mask size to an integer bit mask with the corresponding
	number of bits set to 1. Function-like macro.
	
	Parameters:
		N - A mask size. Should be in the range 0-8 for 8-bit values
			and 0-16 for 16-bit values.
	
	Returns:
		An integer value with only the *N* lowest-significance bits
		set to one (1).
*/
#define BITMASK(N) (BV(N) - BITOPS_1)

/**
	Macro: BITMASK_AT
	Maps a mask size and offset to an integer bit mask with the corresponding
	number of bits set to 1, at the specified offset. Function-like macro.
	
	Parameters:
		N - A mask size. Should be in the range 0-8 for 8-bit values
			and 0-16 for 16-bit values.
		O - A mask offset. Should be less than or equal to (*T* - *N*),
			where *T* is the total number of bits in a value.
	
	Returns:
		An integer value with only the *N* bits starting at bit number *O*
		set to one (1).
*/
#define BITMASK_AT(N, O) (BITMASK(N) << (O))

/**
	Macro: BITMASK_NOT_AT
	Maps a mask size and offset to an inverted integer bit mask with
	the corresponding number of bits set to 0, at the specified offset.
	Function-like macro.
	
	Parameters:
		N - A mask size. Should be in the range 0-8 for 8-bit values
			and 0-16 for 16-bit values.
		O - A mask offset. Should be less than or equal to (*T* - *N*),
			where *T* is the total number of bits in a value.
	
	Returns:
		An integer value with only the *N* bits starting at bit number *O*
		set to zero (0).
*/
#define BITMASK_NOT_AT(N, O) (~BITMASK_AT(N, O))

/**
	Macro: BITSTATES
	Maps a number of bits to the number of possible states of a value with
	that number of bits. Alias of <BV>. Function-like macro.
	
	Parameters:
		N - A number of bits.
	
	Returns:
		The number of possible states of an *N*-bit value (i.e. 2^*N*).
*/
#define BITSTATES(N) BV(N)

/**
	Macro: BITMAX
	Utility macro that maps a number of bits to the maximum value of an
	unsigned integer with that number of bits. Alias of <BITMASK>.
	Function-like macro.
	
	Parameters:
		N - A number of bits.
	
	Returns:
		The maximum value of an *N*-bit unsigned integer (i.e. 2^*N* - 1).
*/
#define BITMAX(N) BITMASK(N)

/**
	Macro: GET_BITFIELD
	Maps a field size and an arbitary integer to an integer containing
	the bitfield with the specified size, from the least significant bits
	of the arbitrary integer. Function-like macro.
	
	Parameters:
		S - A field size. Should be in the range 0-8 for 8-bit values
			and 0-16 for 16-bit values.
		D - An arbitrary integer to get a bitfield from.
	
	Returns:
		An integer value containing the *S* lowest-significance bits
		of *D*. The remaining bits are set to zero (0).
*/
#define GET_BITFIELD(S, D) (BITMASK(S) & (D))

/**
	Macro: GET_BITFIELD_AT
	Maps a field size, a field offset and an arbitary integer to an integer
	containing the bitfield with the specified size at the specified offset
	in the arbitrary integer. Function-like macro.
	
	Parameters:
		S - A field size. Should be in the range 0-8 for 8-bit values
			and 0-16 for 16-bit values.
		O - A field offset. Should be less than or equal to (*T* - *S*),
			where *T* is the total number of bits in a value.
		D - An arbitrary integer to get an offset bitfield from.
	
	Returns:
		An integer value containing the *S* bits starting at bit number *O*
		in *D*, shifted into the lowest-significance bits. The remaining bits
		are set to zero (0).
*/
#define GET_BITFIELD_AT(S, O, D) GET_BITFIELD(S, (D) >> (O))

/**
	Macro: MAKE_BITFIELD_AT
	Maps a field size, a field offset and an arbitary integer to an integer
	that has a bitfield with the specified size at the specified offset,
	containing bits from the arbitrary integer. Function-like macro.
	
	Parameters:
		S - A field size. Should be in the range 0-8 for 8-bit values
			and 0-16 for 16-bit values.
		O - A field offset. Should be less than or equal to (*T* - *S*),
			where *T* is the total number of bits in a value.
		X - An arbitrary integer containing a bitfield value.
	
	Returns:
		An integer value containing the *S* lowest-significance bits of *X*,
		shifted to offset *O*. The remaining bits are set to zero (0).
*/
#define MAKE_BITFIELD_AT(S, O, X) (GET_BITFIELD(S, X) << (O))

/**
	Macro: CLEAR_BITFIELD_AT
	Maps a field size, a field offset and an arbitary integer to an integer
	where the bitfield with the specified size at the specified offset
	in the arbitrary integer has been cleared. Function-like macro.
	
	Parameters:
		S - A field size. Should be in the range 0-8 for 8-bit values
			and 0-16 for 16-bit values.
		O - A field offset. Should be less than or equal to (*T* - *S*),
			where *T* is the total number of bits in a value.
		D - An arbitrary integer to clear a bitfield in.
	
	Returns:
		An integer value where the *S* bits starting at bit number *O*
		are set to zero (0). The remaining bits have the values they have in *D*.
*/
#define CLEAR_BITFIELD_AT(S, O, D) (BITMASK_NOT_AT(S, O) & (D))

/**
	Macro: SET_BITFIELD_AT
	Maps a field size, a field offset and two arbitary integers to an integer
	that has a bitfield with the specified size at the specified offset,
	containing bits from the second arbitrary integer, while the remaining
	bits are equal to the corresponding bits in the first arbitrary integer.
	Function-like macro.
	
	Parameters:
		S - A field size. Should be in the range 0-8 for 8-bit values
			and 0-16 for 16-bit values.
		O - A field offset. Should be less than or equal to (*T* - *S*),
			where *T* is the total number of bits in a value.
		D - An arbitrary integer to set a bitfield in.
		X - An arbitrary integer containing a bitfield value.
	
	Returns:
		An integer value containing the *S* lowest-significance bits of *X*,
		shifted to offset *O*. The remaining bits have the values they have in *D*.
*/
#define SET_BITFIELD_AT(S, O, D, X) (CLEAR_BITFIELD_AT(S, O, D) | MAKE_BITFIELD_AT(S, O, X))

#endif
