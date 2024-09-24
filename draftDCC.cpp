AlazarSetCaptureClock(
  handle, // HANDLE –- board handle
  EXTERNAL_CLOCK_10MHZ_REF, // U32 –- clock source Id
  250000000, // U32 –- sample rate Id or value
  CLOCK_EDGE_RISING, // U32 –- clock edge Id
  10 // U32 –- decimation value
  );
//generate a 25 MS/s sample rate (250 MHz / 10) from a 10 MHz external clock input
//In 10 MHz PLL external clock mode, the ATS9625/ATS9626 can generate a 250 MHz sample clock from an external 10 MHz reference input. 
//Sample data can be decimated by a factor of 1 to 100000.