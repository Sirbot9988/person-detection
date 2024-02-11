# AI-Deck Object Detection
Repo contains: 
1. Training a simple object detection model (MobileNetv2) 
2. Quantising TFLite Model
3. Generating Autotiler Code for AI-Deck implementation


## Installation
Clone into `aideck-gap8-examples/examples/ai/`

## Dataset
Download dataset from: [Roboflow](https://universe.roboflow.com/wk-meyzk/safmc-nus).

Save them in `aideck-gap8-examples/examples/ai/detection/images/`

## Compilation
1. From a terminal with the docker container, or gap_sdk dev environment, in the `aideck-gap8-examples/` folder, execute:
   - ```docker run --rm -v ${PWD}:/module aideck-with-autotiler tools/build/make-example examples/ai/detection clean model build image```

3. Then from another terminal (outside of the container), use the cfloader to flash the example if you have the gap8 bootloader flashed AIdeck. Change the [CRAZYFLIE URI] with your crazyflie URI like `radio://0/90/2M/E7E7E7E726`.
   - ```cfloader flash examples/ai/detection/BUILD/GAP8_V2/GCC_RISCV_FREERTOS/target.board.devices.flash.img deck-bcAI:gap8-fw -w [CRAZYFLIE URI]```
   - ```cfloader flash examples/ai/detection/BUILD/GAP8_V2/GCC_RISCV_FREERTOS/target.board.devices.flash.img deck-bcAI:gap8-fw -w radio://0/90/2M/E7E7E7E726```

When the example is flashing, you should see the GAP8 LED blink fast, which is the bootloader. The example itself can be noticed by a slow blinking LED. You should also receive the detection output in the cfclient console.
