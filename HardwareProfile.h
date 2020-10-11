// Clock frequency values
#define SYS_FREQ (80000000L)
#define GetSystemClock()		SYS_FREQ			// Hz
#define GetInstructionClock()	(GetSystemClock()/1)	//
#define GetPeripheralClock()	(GetSystemClock()/1)	// Divisor is dependent on the

#define mLED              LATDbits.LATD6
