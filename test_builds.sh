#!/bin/bash

CONFIGS="axoloti/ggm
stm3210e-eval/nsh
stm3240g-eval/nxterm
stm32f4discovery/nxlines
stm3220g-eval/nxwm
stm32f429i-disco/nsh
"

# mikroe-stm32f4/

for cfg in $CONFIGS; do
  echo "**** Building $cfg ****"
  make BOARD_CONFIG=$cfg clean
  make BOARD_CONFIG=$cfg
done
