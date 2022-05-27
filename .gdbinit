target extended-remote :3333
file build/main.elf
load

b main
# b HAL_GPIO_EXTI_Callback

