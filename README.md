# ft60x_driver

#### Build
```
make
```

#### Load
```
sudo insmod ft60x.ko
```

#### Example
With Default FT60x configuration:
* FIFO Mode 245
* 1 Channel

```
$ dmesg

[ 9462.813651] usb 2-1: new SuperSpeed Gen 1 USB device number 2 using xhci_hcd
[ 9462.831246] usb 2-1: New USB device found, idVendor=0403, idProduct=601f, bcdDevice= 0.00
[ 9462.831254] usb 2-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[ 9462.831258] usb 2-1: Product: FTDI SuperSpeed-FIFO Bridge
[ 9462.831262] usb 2-1: Manufacturer: FTDI
[ 9462.831266] usb 2-1: SerialNumber: 000000000001
[ 9462.835717] FT60x i/f 0 now probed: (0403:601F)
[ 9462.835721] ID->bNumEndpoints: 02
[ 9462.835723] ID->bInterfaceClass: FF
[ 9462.835725] ED[0]->bEndpointAddress: 0x01
[ 9462.835727] ED[0]->bmAttributes: 0x02
[ 9462.835729] ED[0]->wMaxPacketSize: 0x0400 (1024)
[ 9462.835732] ED[1]->bEndpointAddress: 0x81
[ 9462.835734] ED[1]->bmAttributes: 0x03
[ 9462.835737] ED[1]->wMaxPacketSize: 0x0040 (64)
[ 9462.836316] FT60x i/f 1 now probed: (0403:601F)
[ 9462.836319] ID->bNumEndpoints: 02
[ 9462.836321] ID->bInterfaceClass: FF
[ 9462.836324] ED[0]->bEndpointAddress: 0x02
[ 9462.836326] ED[0]->bmAttributes: 0x02
[ 9462.836331] ED[0]->wMaxPacketSize: 0x0400 (1024)
[ 9462.836334] ED[1]->bEndpointAddress: 0x82
[ 9462.836336] ED[1]->bmAttributes: 0x02
[ 9462.836338] ED[1]->wMaxPacketSize: 0x0400 (1024)
[ 9462.836458] ft60x 2-1:1.1: USB FT60x device now attached to ft60x-0
```

```
$ ls /dev/ft60x0 
/dev/ft60x0
```
