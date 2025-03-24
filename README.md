# AI-Deck Object Detection

![(http://url/to/img.png)](https://github.com/cweekiat/detection/blob/main/asset/Screenshot%20from%202024-02-19%2020-52-23.png)

Repo contains: 
1. Training a simple object detection model (MobileNetv2) 
2. Quantising TFLite Model
3. Generating Autotiler Code for AI-Deck implementation

## Quickstart
1. Clone repository into `aideck-gap8-examples/examples/ai/` folder
2. From a terminal with the docker container, or gap_sdk dev environment, in the `aideck-gap8-examples/` folder, execute:
   - ```docker run --rm -v ${PWD}:/module aideck-with-autotiler tools/build/make-example examples/ai/person-detection clean model build image```

This takes a while to complete. Here, check the Autotiler output to see memory usage of ONLY the CNN. 
If the occupation of L2 seems too high (ie. ~90% of 270000 Bytes occupied) then there is a high likelyhood the program will run out of memory during run time.

Some things to keep in mind: 
- Image buffer (JPEG) takes ~70kb
- Flashed content lives on L2.
- Parallel cores cannot access cluster content. 

3. Flashing

### CFLOADER (Radio)

From another terminal (outside of the container), use the cfloader to flash the example if you have the gap8 bootloader flashed AIdeck. Change the [CRAZYFLIE URI] with your crazyflie URI like `radio://0/100/2M/E7E7E7E711`.
   - ```cfloader flash examples/ai/person-detection/BUILD/GAP8_V2/GCC_RISCV_FREERTOS/target.board.devices.flash.img deck-bcAI:gap8-fw -w radio://0/100/1M/E7E7E7E711```
   - Flashing will take ~10 minutes depending on the model size and it may appear to be stuck at 99%. However, if the crazyflie automatically reboots at 99%, the flashing is usually completed and you can `Ctrl+C` in the terminal.  
   - Reboot the Crazyflie.

When the example is flashing, you should see the GAP8 LED blink fast, which is the bootloader. The example itself can be noticed by a slow blinking LED. You should also receive the detection output in the cfclient console.

### JTAG
Connect the JTAG through the left (GAP8 input). 

Then run 
```sudo docker run --rm -v ${PWD}:/module --privileged aideck-with-autotiler tools/build/make-example examples/ai/person-detection/ all run ```