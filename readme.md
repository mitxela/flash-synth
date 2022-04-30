# Flash Synth

Background reading: https://mitxela.com/projects/flash_synth

The processor is an STM32L432KC, running at 64MHz (determined experimentally to give the best performance when powered by MIDI). The chip has 256kB of flash memory and 64kB of RAM.

MIDI input on USART1, interrupting on every byte, no buffer.

Timer6 and DMA are used to feed samples to the 12-bit DAC, interrupts at half-transfer and transfer-complete are where samples are generated. Switching between stereo and mono sound output requires reinitializing the DMA.

HAL functions are used for some of the setup, there is a small runtime overhead associated with HAL, a future task is to rewrite this using register access or the LL functions.

Wavetables are 8192 samples long. All oscillator phases are normalized from 0 to 8192.0, even the algorithms which don't use tables.

12-bit DAC means 4096 output levels, at a polyphony of 16 this means a maximum amplitude of ±128.

Pretty much everything that can be cached, is cached. MIDI data says a certain parameter must be X, and we need X\*K in the algorithm, then multiply by K at the moment it arrives. RAM is much cheaper than processor cycles.

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

Note: I have now added more detailed instructions [towards the end of this page](#detailed-compilation-instructions).

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


## Detailed compilation instructions

Note: since this github repo is currently private, you need to authenticate to be able to clone it. Github no longer allows password auth for cloning so you will need to import (or generate) an SSH key, and add the public part to your github profile. There are numerous guides online for how to do this. If/when we make this repo public, this won't be needed.

### Compiling on Linux (Debian / Ubuntu)
1. Install, update and upgrade apt.
   ```
   sudo apt install
   sudo apt update
   sudo apt upgrade
   ```

2. Install the ARM GCC toolchain. On Ubuntu and Debian, the package name is `gcc-arm-none-eabi` 
   ```
   sudo apt-get install gcc-arm-none-eabi
   ```

3. If you don't already have it, install `make` which will be part of the essential build tools (`build-essential` on Debian, `base-devel` on Arch).
   ```
   sudo apt install make
   ```

4. Clone the repo, `cd` into it and run `make` to compile. (Reminder: see the note about credentials above under Detailed compilation instructions about cloning the repo)
    ```
    git clone https://github.com/mitxela/flash-synth
    cd flash-synth
    make
    ```

If everything worked, you should now have a `build` folder, containing `flash-synth.bin`, along with `flash-synth.pgm` (the memory preview image) and a whole load of intermediate build files.

The file named `flash-synth.bin` is the compiled firmware file.

If using WSL2, see the extra step below for copying the binary file from the virtual machine to your Windows file system.

### Compiling on Arch Linux
Infuriatingly, the ARM GCC toolchain is inconsistently named in different places. On Arch, the package name is `arm-none-eabi-gcc`.

You will also need to install git and xxd (which is normally packaged as part of Vim, but on Arch it's also available standalone in the AUR). If you don't already have it, install `make` which will be part of the essential build tools (`build-essential` on Debian, `base-devel` on Arch).
```
sudo pacman -S base-devel arm-none-eabi-gcc git vim
```

Then clone the repo, `cd` into it and run `make` to compile.

### Compiling on Windows (WSL)

WSL gives you a Linux installation on windows. Currently by default this means Ubuntu.

There are two versions of WSL, the instructions are the same for either version. The main differences between the versions are file handling and speed. WSL2 is installed by default using the instructions below.

Do a Windows search for "command prompt" (you __must__ do this to run as administrator). Right-click on the result and choose "Run as administrator", then type:
```
wsl --install
```

Once installed, the compilation instructions are identical to building on Linux (see above Compiling on Debian / Ubuntu).

If you want to work with this file under Windows, you will need to copy the binary from the virtual machine to your Windows file system. Note that this is __not__ done in WSL/Linux, it is done using Windows PowerShell or command prompt. Below, we copy the file to the desktop.

Open a command prompt as Administrator again then type:
```
copy \\$wsl\Ubuntu\home\<your-wsl-linux-username>\flash-synth\build\flash-synth.bin C:\Users\<your-windows-username>\Desktop
```

Note: if you are using WSL 1, the drives are mounted within the environment, the C: drive is at `/mnt/c/` so to do the above in a WSL 1 terminal:
```
cp build/flash-synth.bin /mnt/c/Users/<username>/Desktop
```
However, if using WSL 1 I would recommend simply cloning into a folder on the C: drive to begin with, e.g.
```
cd /mnt/c/Users/<username>/Documents
git clone https://github.com/mitxela/flash-synth
cd flash-synth
make
```

### Compiling on Windows (MSYS2)

If you're not interested in WSL then the best way to compile is probably MSYS2. I really like MSYS2, it uses pacman (as in Arch Linux) and generally works very well. Here's some step-by-step instructions:

1. Start by heading to https://www.msys2.org/ and follow the installation instructions, which means downloading and running the installer, then running `pacman -Syu` etc. After installing, you may want to go to the options (right click the title bar) and under `Keys`, select `Ctrl+Shift+letter shortcuts`. This lets you paste by pressing ctrl+shift+V. 

2. In the MSYS terminal, install the necessary software by running:
   ```
   pacman -S base-devel git vim mingw-w64-x86_64-arm-none-eabi-gcc
   ```
   (the `xxd` package is bundled with vim)

   2.5 Extra step: On my MSYS install, the tools were installed to somewhere that wasn't included in PATH. Try running `arm-none-eabi-gcc` on its own - if you get "fatal error: no input files" then it's working. If you get "command not found" then you need to add it to your PATH. The simplest way is to run this command once:
   ```
   echo 'export PATH=/mingw64/bin:$PATH' >> ~/.bashrc
   ```
   and then restart the MSYS terminal.

3. Clone this repo. MSYS puts the C: drive at `/c/` (Whereas WSL puts the C: drive at `/mnt/c/`) but you can just clone into the msys home directory if you prefer, which is wherever you installed it (e.g. `C:\msys64\home\User`)
   ```
   git clone https://github.com/mitxela/flash-synth.git
   ```
   (Reminder: see the note about credentials above under Detailed compilation instructions)

4. Enter the directory by typing 
   ```
   cd flash-synth
   ```

5. Compile the code by typing
   ```
   make
   ```
If everything worked, you should now have a build folder, containing `flash-synth.bin`, along with `flash-synth.pgm` (the memory preview image) and a whole load of intermediate build files.

### Updating

If the code is updated and you want to download the latest version and compile it, within the flash-synth directory run:
```
git pull
```
and then
```
make
```
It will only re-compile the bits needed. If you want to recompile from scratch, run `make clean` first.
