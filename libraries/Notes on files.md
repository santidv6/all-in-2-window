**Notes on libraries files:**

+ The sensorlib2.h and sensorlib2.lib files are header and precompiled library that includes all the necessary code from individual libraries to use the BOOSTXL-SENSORS Boosterpack with a TM4C1294 Connected Launchpad.
  
  The modifications applied to the original resources are developed as stated below:
  - OPT3001.h comes from MSP432-SimpleLink HAL_OPT3001.h/.c files modified for returning lux in float type following the implementation of opt3001.c file from boostxl_sensors example folder.
  The exact path to the original files are:
  \simplelink_msp432p4_sdk_3_40_01_02\examples\nortos\MSP_EXP432P401R\demos\boostxl_edumkii_lightsensor_msp432p401r (for HAL_OPT3001.h/.c)
  and
  \simplelink_msp432p4_sdk_3_40_01_02\examples\nortos\MSP_EXP432P401R\demos\boostxl_sensors_sensorgui_msp432p401r (for opt3001.c)

  - HAL_I2C.h comes from MSP432-SimpleLink boostxl_edumkii_lightsensor example HAL_I2C.h/.c files but modified for working with Tiva Launchpad.

  - BOSCH_datatypes.h comes from a section of MSP432-SimpleLink typedef.h file from boostxl_sensors_sensorgui example.
  
    SimpleLink url for reference: https://www.ti.com/tool/download/SIMPLELINK-MSP432-SDK

+ FT800_TIVA2.h/.c based on FTDI's AN_275 and modified accordingly to the needs of the project.

+ iofuncs.h is a simple library made from scratch.