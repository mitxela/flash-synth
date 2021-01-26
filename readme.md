# Flash Synth

Background reading: https://mitxela.com/projects/flash_synth

The processor is an STM32L432KC, running at 64MHz (determined experimentally to give the best performance when powered by MIDI). The chip has 256kB of flash memory and 64kB of RAM.

MIDI input on USART1, interrupting on every byte, no buffer.

Timer6 and DMA are used to feed samples to the 12-bit DAC, interrupts at half-transfer and transfer-complete are where samples are generated. Switching between stereo and mono sound output requires reinitializing the DMA.

HAL functions are used for some of the setup, there is a small runtime overhead associated with HAL, a future task is to rewrite this using register access or the LL functions.

Wavetables are 8192 samples long. All oscillator phases are normalized from 0 to 8192.0, even the algorithms which don't use tables.

12-bit DAC means 4096 output levels, at a polyphony of 16 this means a maximum amplitude of ±128.

Pretty much everything that can be cached, is cached. MIDI data says a certain parameter must be X, and we need X*K in the algorithm, then multiply by K at the moment it arrives. RAM is much cheaper than processor cycles.

Envelopes are all optimised by caching their deltas, so the per-sample code only consists of an addition. This means they must be quantised to the buffer size. This is done in the inlined envelope functions.

Defines for POLYPHONY and BUFFERSIZE are there for clarity only, things will break if they are changed.

### Notes on FM algorithms

In algorithms 1 and 2, for the sake of extreme optimisation, no interpolation at all is done between samples. Instead, the wavetables are just about big enough to let us pre-process them with an antialiasing filter, making a nearest-neighbour interpolation viable. There is some loss of bit-depth at the low end, which can be heard, but the benefit is we save at least two multiply-accumulates per oscillator per sample. I wasn't sure if I was going to get away with this, but so far no-one has complained.

Similarly there is no explicit antialiasing at the top end, only the pre-processing of the wavetables. The FM effect depth is reduced with higher pitches to limit aliasing.

For reduced-polyphony algorithms, variables of the "unused" voices are re-used for various things. Clarity could potentially be improved by using macros for some of this.

### Notes on Algo 3

The bleps-via-state-machine has the same output as a 2-sample polyblep, but with a fraction of the overhead. Looking through the commit history may help to explain how it works.

The constant for filter tracking was determined experimentally, and may not be correct (or even linear).


## Compiling

`arm-gcc` and a posix environment are needed to compile. An ST-link utility is needed to flash the dev board. `make flash` assumes a WSL environment, use `make flash-stl` on linux. The makefile could be updated to detect OS.

`xxd` (command line hex editor) is needed by `version.sh` (post-build step)

It's possible to attach GDB for debugging, some effort would be required to get this to work with an IDE.

In the build folder, a .pgm file is generated which gives a visual overview of program memory usage.

## File descriptions

- `Src/main.c` is the main source code, including entry point, usart and DMA interrupts

- `Src/algo3_mono.h` and `Src/algo3_poly16.h` are template files for algorithm 3. They are included repeatedly with different defines to limit the amount of code repetition without adding overhead. This method of doing it was settled on after much debate.

- `Src/system_stm32l4xx.c`, `Src/stm32l4xx_hal_msp.c`, `Src/stm32l4xx_it.c`, and the contents of `Inc/` are part of the generated setup code, do not add new interrupts into `_it` files, stick them in `main.c`. These files could potentially be removed/merged into `main.c`.

- `Src/lookupTables.h` contains the main lookup tables and constants data. Some of the tables have javascript snippets which were used to generate them. Anything with a section attribute has a specific place it needs to be in memory, to be updated with the bootloader/programming software, in many cases the data itself is just a placeholder.

- `Src/defaultPatches.h` contains more constants data related to the patches and a config section. The config section could potentially be formatted better, there isn't a pleasant way to stick individual variables at specific locations in gcc.

- `version.sh` is invoked by the makefile and inserts the version number into the compiled binary. The location is amidst the interrupt vectors (labelled as "reserved area" in `startup_stm32l432xx.s`). Having this as a separate file makes it easier to keep track of. It also inserts a commit hash if the working tree is clean or a timestamp otherwise.


## Potential optimisations / improvements

- Remove remaining HAL code
- SysTick is not used, should be stripped out if there's no HAL function left depending on it

- RAM usage, we're OK for RAM at the moment but several chunks are only used in certain modes, if we get short of RAM these should be unionised. The cleanest way of doing this I can think of is to use macros.

- For clarity, more of the repeated code relating to algorithms 1 and 2 should be moved to the newer template format.

- FM depth adjustment in realtime, implement similarly to how the filter in algo 3 is done.

- (daydreaming) For a potentially enormous performance increase, move all executed code to RAM, and replace all function pointer stuff with assembled code segments. This will require re-writing a vast amount of the synth code. There may be some benefits of moving interrupts into RAM regardless, this is not difficult to do, but hard to profile.


## Known problems

- bitfield concurrency issues. Some further investigation of the preemption priority levels is needed to see what's interrupting what, but it's become apparent that bitfields under gcc are not thread-safe. The only thing still using bitfields is the envelope state. The effect is that very occasionally, an envelope skips its attack phase. It should be possible to fiddle with things so that problems never happen. Worst case scenario, use an entire byte for every boolean, but there should be a simpler solution than that.

- bootloader distortion problem with original 150 units


## Development board

STM produce development boards for most STM32 chips. The great thing is that they subsidise the cost of them, especially the nucleo boards which often cost less than the parts that are on them.

The primary advantage of using a dev board is that a USB J-link programmer is built in, so you can quickly flash a new firmware image without having to unplug the MIDI cable.

The NUCLEO-L432KC board can be used to run the flash synth pretty much as-is. If you are sending MIDI data from a keyboard powered by the same source as the USB port on the nucleo board, then a single 2.2k pullup resistor on the midi data is all that's needed.

On the board, the DAC outputs are labelled A3 and A4. MIDI data goes to UART1_RX, labelled D0/RX on the nucleo board.

![Dev board pinout](https://os.mbed.com/media/uploads/bcostm/nucleo_l432kc_2017_10_09.png)
https://os.mbed.com/platforms/ST-Nucleo-L432KC/

To match the hardware closer, the DAC outputs should have a high pass filter and increased output impedance. The real hardware has this to limit the power delivered when low impedance headphones are connected, directly connecting them to the DAC buffer would brownout the chip.

Originally I threw together a carrier using some protoboard like this:
![original carrier board](https://mitxela.com/img/uploads/tinymidi/flash/L432devboard.jpg)

Eventually that wore out so I made a new carrier, which has proper ground isolation:
![new carrier board](https://mitxela.com/img/uploads/tinymidi/flash/devboard2.jpg)

The nucleo board does not break out the BOOT0 pin. The blue wire in the picture above is soldered directly to the BOOT0 pad. This isn't necessary unless you want to debug the bootloader.

Since the dev board is powered by USB, it is possible to up the clock speed and draw more power than would be normally available. I recommend testing on real hardware after any substantial change.

I only populated one of these but I have another 9 spare boards I could share.
