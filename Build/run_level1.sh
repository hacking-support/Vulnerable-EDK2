#!/bin/bash

set -e

function cleanup {
  rm -f /tmp/guest.in /tmp/guest.out
}

trap cleanup EXIT

mkfifo /tmp/guest.in /tmp/guest.out

$HOME/build/qemu-8.2.4/build/qemu-system-x86_64  \
  -drive if=pflash,format=raw,readonly=on,file=./OvmfX64/DEBUG_GCC5/FV/OVMF_CODE.fd \
  -drive if=pflash,format=raw,file=./OvmfX64/DEBUG_GCC5/FV/OVMF_VARS.fd -hda fat:raw:rw:./VulnerableCode/DEBUG_GCC5/X64/  \
  -net none -serial mon:stdio -serial pipe:/tmp/guest -d pcall
