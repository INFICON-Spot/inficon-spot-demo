# inficon-spot-demo
INFICON Spot Demonstrator Kit firmware

This Arduino sketch is the firmware for the Spot Demonstrator kit described in
the manual that is available for download at
https://products.inficon.com/en-us/nav-products/product/detail/spot-cds500d/

## Required hardware
* Arduino Uno (tested on Rev3 SMD)
* Adafruit 716 LCD Shield (https://www.adafruit.com/product/716)
* INFICON Spot Sensor (Spot CDS500D or CDS530D)

## Hardware connection to Spot
| Arduino           | Spot           |
|-------------------|----------------|
| +5V               | +5V (Pin 1)    |
| GND               | GND (Pin 2)    |
| Pin 13            | Clk (Pin 3)    |
| Pin 10            | SS/ (Pin 4)    |
| Pin 12            | MISO (Pin 5)   |
| Pin 11            | MOSI (Pin 6)   |
| Pin 3             | RDY/ (Pin 7)   |

Pins 8, 9 and 10 of the Spot are left unconnected.

# Github
The project is available on github at
https://github.com/criesch/inficon-spot-demo

# Bugs
Please report bugs to Christian Riesch <christian.riesch@inficon.com>
