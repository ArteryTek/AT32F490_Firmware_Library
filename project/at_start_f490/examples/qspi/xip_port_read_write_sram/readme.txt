/**
  **************************************************************************
  * @file     readme.txt 
  * @brief    readme
  **************************************************************************
  */

  this demo is based on the at-start board, in this demo, systick used for 
  delay function. qspi xip port operate write/read psram, if qspi test pass,
  led3 fresh, else led2 fresh.

  the qspi sram is aps1604m. 
  - qspi io0   --->   pd13
  - qspi io1   --->   pd11
  - qspi io2   --->   pd12
  - qspi io3   --->   pd15
  - qspi sck   --->   pd14
  - qspi cs    --->   pd10

  for more detailed information. please refer to the application note document AN0088.

