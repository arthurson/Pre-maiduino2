it is a fork from Pre-maiduino.

it builds with Arduino IDE 2.3.7 and Arduino Core for STM32 2.12.0.

all codes are written by deepseek ai.



v0.01a 25-2-2026.

firmware can be complied, all servos can be set homepoint, thus the premaid ai can stand up finally.

rn42 bluetooth can be used as serial input, it can show mpu6050 gyro data, led has breathing light, basically all hardwares can be used at the moment.
the script has some sample actions, it works and work really ugly.

for this version stm32f102c8 will run 72mhz, have 20kb sdram and use 128kb flash, kind of overclocking, this firmware is 49kb, very bulky.

the next step is to find out all servo safe point, the data from 『楽しく遊ぶプリメイドAI』is not accurate enough, and try to port action from other arduino humanoid like plen2.
