# Motor controller Manta50

This project requires TI Motorware 18. Adjust the paths in project settings where necessary.

The motor controller control via CanOpen.

### link to hardware project [manta50-80](https://github.com/ipmgroup/manta50-80)
#### Many thanks to Oleksandr Novychenko for the implementation of the small CanOpen library.

To configure and control on desktop PC or RPi use [Canfestival](https://github.com/ipmgroup/canfestival) and [WEB GUI](https://github.com/ipmgroup/thruster_gui).

We used a controller for ROV UAV and drones. If you need low engine speeds - this is the best.
##### install TI Tools
I had hope that TI would do a GIT. But that didn't happen.

because you can download the tools yourself from TI.

You need to install Motorware 18, Unuflash, CGT.

for Motorware use wine.

    sudo apt install wine:i386
    wine motorware_1_01_00_18_setup.exe
    chmod +x ti_cgt_c2000_22.6.1.LTS_linux-x64_installer.bin uniflash_sl.8.5.0.4593.run
    ./ti_cgt_c2000_22.6.1.LTS_linux-x64_installer.bin --mode unattended --prefix ~/ti
    ./uniflash_sl.8.5.0.4593.run --mode unattended --prefix ~/ti/uniflash
    cp -r ~/.wine/drive_c/ti/motorware/ ~/ti/

if you have a warning
##### "Failed to locate system libraries required for UniFlash operation:"
install the necessary libraries.
##### Build project
    mkdir build
    cd build
    cmake ..
    make
    make flash
