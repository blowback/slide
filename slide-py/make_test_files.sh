#!/bin/bash

# zero length file, pathological test case
touch TEST0.dat

# small files exercise single-frame transfer
dd if=/dev/urandom of=TEST256.dat bs=256 count=1
dd if=/dev/urandom of=TEST512.dat bs=512 count=1
dd if=/dev/urandom of=TEST1K.dat bs=512 count=2

# exercise multi-frame transfer
dd if=/dev/urandom of=TEST2K.dat bs=512 count=4
dd if=/dev/urandom of=TEST4K.dat bs=1K count=4
dd if=/dev/urandom of=TEST8K.dat bs=1K count=8
dd if=/dev/urandom of=TEST16K.dat bs=1K count=16

# bigger than a page
dd if=/dev/urandom of=TEST32K.dat bs=1K count=32
dd if=/dev/urandom of=TEST64K.dat bs=1K count=64

# bigger than address space
dd if=/dev/urandom of=TEST128K.dat bs=1K count=128

# bigger than RAMDISK (242 Kbytes)
dd if=/dev/urandom of=TEST256K.dat bs=1K count=256
