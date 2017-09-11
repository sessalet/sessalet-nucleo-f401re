# eclipsews-asp-nucleof401re

## asp/target/nucleo_f401re_gcc/Makefile.target の修正

`KERNEL_COBJS :=` に以下を追加してください。

`stm32f4xx_hal_i2c.o stm32f4xx_hal_tim.o stm32f4xx_hal_tim_ex.o`

## asp/target/nucleo_f401re_gcc/stm32fcube/stm32f4xx_hal_conf.hの修正

`#define HAL_I2C_MODULE_ENABLED` を有効にしてください。
