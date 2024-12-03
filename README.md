# AI-Deck Object Person Detection

Forked from [cweekiat/detection](https://github.com/cweekiat/detection)

Repo contains: 
1. Generating Autotiler Code for AI-Deck implementation

## Quickstart
1. Clone repository into `aideck-gap8-examples/examples/ai/` folder
2. From a terminal with the docker container, or gap_sdk dev environment, in the `aideck-gap8-examples/` folder, execute:
   - ```docker run --rm -v ${PWD}:/module aideck-with-autotiler tools/build/make-example examples/ai/person-detection clean model build image```
3. From another terminal (outside of the container), use the cfloader to flash the example if you have the gap8 bootloader flashed AIdeck. Change the [CRAZYFLIE URI] with your crazyflie URI like `RADIO URI`.
   - ```cfloader flash examples/ai/detection/BUILD/GAP8_V2/GCC_RISCV_FREERTOS/target.board.devices.flash.img deck-bcAI:gap8-fw -w RADIO_URI```
   - Reboot the Crazyflie.

When the example is flashing, you should see the GAP8 LED blink fast, which is the bootloader. The example itself can be noticed by a slow blinking LED. You should also receive the detection output in the cfclient console.