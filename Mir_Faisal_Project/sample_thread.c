
initramfs
=================

Can be loaded from an arbitrary location ( most often tftp ) and is self-contained in memory.
This is like a Linux LiveCD. Often used to test firmware, as the first part of a multi-stage
installation or as a recovery tool. 3)
An initramfs and initrd are basically the same. It’s a filesystem in memory, which contains 
userland software. In an embedded environment it might contain the whole distro, on bigger 
systems it can contain tools&scripts to assemble&mount raid arrays and stuff like that before
passing userland boot to them. Both can have a uHeader, to let uBoot know what it is.

The initramfs-kernel image is used for development or special situations as a one-time boot as a
stepping stone toward installing the regular sysupgrade version. Since the initramfs version runs
entirely from RAM, it does not store any settings in flash, so it is not suitable for operational use.

initramfs-uImage.bin: initramfs-kernel.bin: