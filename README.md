# Compilation

'testing' folder can be put it in examples folder and existing Makefile can be used to compile binaries. If correct SDK path is provided source files can also be compiled under different environments.

Before QEMU starts qemu_comm.py (router script) must be running. When QEMU starts, script will print information about the new machine.

There are two programs that come with separate Makefiles:

main.c: This should be started first in QEMU. (device id: 0xbeef)

gpio_peer.c: This should be started within few seconds after main program. (device id: 0xface)

# Example command line to run binaries

main:
qemu-system-arm -M nrf51 -nographic -nrf-id 0xbeef -kernel <program.bin>

gpio_peer:
qemu-system-arm -M nrf51 -nographic -nrf-id 0xbeef -kernel <program.bin>

# Read UART console

socat - UNIX:/tmp/nrf51_beef.sock

socat - UNIX:/tmp/nrf51_face.sock
