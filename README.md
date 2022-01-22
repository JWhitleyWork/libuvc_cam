# libuvc_cam: A ROS2 Camera Driver Based on [libuvc](https://github.com/libuvc/libuvc)

This camera driver supports all cameras which are classified by the operating system as
UVC (USB Video Class) devices. It utilizes [libuvc_vendor](https://github.com/JWhitleyWork/libuvc_vendor),
a ROS2 wrapper for the [libuvc](https://github.com/libuvc/libuvc) project.

As opposed to most camera device drivers for ROS, this driver expects a vendor and product ID
and optional serial number to identify the camera. This allows for easy management of multiple
USB cameras of the same model which can be differentiated by serial number without having to manage
device paths (which are usually automatically assigned and can change). However, this requires the
device manufacturer to have provided the device serial number in the firmware which is not always
the case.

## Device Identification
In Linux systems, you can identify USB cameras which are attached to the system with the command
`lsusb`. Example output is provided below:

```
Bus 004 Device 002: ID 2a0b:00b8 Leopard Imaging LI-GW5200
Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 003 Device 002: ID 5986:9102 Acer, Inc BisonCam,NB Pro
Bus 003 Device 004: ID 8087:0026 Intel Corp. 
Bus 003 Device 003: ID 06cb:00c7 Synaptics, Inc. 
```

Two cameras are present on this system: One from Leopard Imaging and one from Acer, Inc. The
associated vendor and product IDs are found after the "ID" and are in the form of
`vendor_id:product_id`. For example, the Leopard Imaging LI-GW5200 has a vendor ID of `2a0b` and a
product ID of `00b8`. To find the serial number of the device, we can run
`sudo lsusb -v -d 2a0b:00b8 | grep Serial`. In this command, `-v` means "verbose", `-d` specifies
the device, and `| grep Serial` means "pass the output of the previous command to `grep` and only
return lines that contain the word `Serial`. `sudo` is required because getting the device serial
number requires opening the device, which only `root` can do until you set up the `udev` rules as
described in the next section. Example output:

```
  iSerial                 3 0000000001
can't get debug descriptor: Resource temporarily unavailable
```

In this case, the device serial number is `0000000001` (I told you some manufacturers don't set
this correctly, didn't I?). Ignore the `3` or similar number followed by a space before the serial
number as this is just a quirk of how `lsusb` displays the information.

## Device Access
As opposed to `v4l2` and OpenCV drivers, accessing a device at the `libusb` level (the USB library
utilized by `libuvc`) requires extra permissions. However, once you know the vendor ID and product
ID, setting these permissions is relatively straight-forward. In this repository, you will find a
file named `example.rules`. If you installed this package from the ROS `apt` repository, it will be
in `/opt/ros/<distro_name>/share/libuvc_cam`. To properly set the permissions for your device,
modify this file with your device's vendor ID in place of `xxxx` (both locations) and your device's
product ID in place of `yyyy` (both locations). After this, use root permissions to copy it to
`/etc/udev/rules.d/` and rename it with the following command - replacing `vendor` with your
camera's vendor name:

```
sudo cp example.rules /etc/udev/rules.d/vendor.rules
```

Once this is complete, either un-plug and re-plug your camera(s) or run `sudo udevadm trigger` to
apply the new permissions.
