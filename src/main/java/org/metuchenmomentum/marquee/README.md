# The Marquee Control Library

Welcome to the Marquee Control Library, which is used to send commands
from the RoboRio to the ESP32 that controls the 8 x 32 WS2812B
individually addressable LED array. 

## Overview

The library provides four classes that sends commands via an SPI connection
from the RoboRio to the ESP32. The library provides the following
classes:

* `DisplayCommand`, an `enum` of supported display functions, ranging
  from single pixel display to a full text crawl
* `DisplayMessage`, the messsage that the RoboRio sends to the ESP32.
  The message includes foreground and background colors, text to display,
  and other information. Please see the class documentation for details.
* `DisplayConnection`, which specifies the message sending API.
* `SpiDisplayConnection`, a `DisplayConnection` implementation that
  sends messages via a
  [Serial Peripheral Interface](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface)
  (SPI) connection.

## Using the API

An Arduino IDE is needed to install the Marquee software on the ESP32.
We have used
[Sloeber](https://eclipse.baeyens.it/),
an
[Eclipse](https://www.eclipse.org/)-based
IDE. The standard
[Arduino](https://www.arduino.cc/)
[IDE](https://www.arduino.cc/en/software)
will install the software, but the project is too large to use it for
serious development. There are probably other candidates. Use the one
that suits you. Whichever you choose, be sure to install the ESP32
Arduino libraries and to generate code for the ESP32 S2.

To use the api:

1. Install an Arduino IDE onto 
2. Wire the ESP32 to the Display Panel as described in the
   [Marquee Library Documentation](https://github.com/frc-team7587/Marquee-1/blob/main/TestLedMatrix/README.md#wiring-the-display-panel).
3. Wire the RoboRio to the ESP32 as described in the
   [Marquee Library Documentation](https://github.com/frc-team7587/Marquee-1/blob/main/TestLedMatrix/README.md#wiring-the-display-panel).
   If you are using both the 5 Volt and 3.3 Volt buck converters,
   **DO NOT MIX THEM UP** as you will destroy the ESP32 if you do.
   
4. Incorporate the Marquee library into the robot control software.
   Invoke the API as desired.
5. Enjoy the fruits of your labor.

## Classes

The library provides the following classes that work together to drive
the marquee

### `DisplayCommand`

The `DisplayCommand` enumeration specifies the Marquee's supported
actions. Actions range from moving a single pixel around the array,
features that mainly support hardware level debugging, to a full fledged
text crawl.

:warning: keep the `DisplayCommand` enumeration in sync with the
C++ enum specified in Marquee library's
[`DisplayCommand.h`](https://github.com/frc-team7587/Marquee-1/blob/main/TestLedMatrix/DisplayCommand.h)
file. If you change one, you **must** make the corresponding change in the other.

### `DisplayConnection`

The `DisplayConnection` interface specifies the API that sends messages
to the ESP32. It contains one function that sends a command.

### `DisplayMessage`

The `DisplayMessage` class simplifies command generation. It provides
convenient defaults for all fields, zero for numeric values and the
empty string for text values, and allows the users to set:

* Text to display, if any
* Display command, what the marquee should do with the provided
  values
* Foreground Red, Green, and Blue, the components of the foreground
  color. Note that text is displayed in the foreground color.
* Background Red, Green, and Blue, the components of the background
  color. 
* Delay 1 and Delay 2, delays to apply to the display. Delay meanings
  vary by command. For scrolling text, delay 1 specifies the dwell time
  for the initial display, and delay 2 specifies the interval between
  crawl shifting.

:warning: It is strongly recommended not to set any unsused field values
so as to maximize backward compatility if currently unused
values are brought into service.

Color intensities vary from 0 (completely off) to 255 (full power)
and default to 0. Delays are in ESP32 ticks, where one tick is one
millisecond, and default to 0.

Please remember that a single 8 x 32 LED panel consumes 75 watts
when run at full power. The panel is way too bright at full power,
and running at full power drastically shortenes the panel's
life. Please use accordingly.

Commands are documented below.

Display commands are sent as pipe delimited character sequences of
128 or fewer bytes. The sequence includes the command,
display text (if any), foreground and background colors,
and delays. 

:warning: the LED matrix consumes 75 watts when run at full power,equivalent to 750 watts incandescent. Running at full power
significantly shortens LED lifetime. Select RGB intensities
accordingly.

### `SpiDisplayConnection`

The `SpiDisplayConnection` class is a
[`DisplayConnection`](#displayconnection)
implementation that sends messages over
[SPI](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface).
The channel is (i.e. slave select pin) is set at construction.

### `UsbDisplayConnection`

The `UsbDisplayConnection` class is a 
[`DisplayConnection`](#displayconnection) 
implementation that sends commands over a USB serial connection,
a USB connection that emulates a asychronous serial terminal,
a.k.a. a serial port. 

### `UsbSerialDeviceList`

The `UsbSerialDeviceList` finds all serial USB devices, file
paths of the form `/dev/ttyUSBn`, where `n` is 0, 1, or 2.

### `UsbSerialPortBuilder`

The `UsbSerialPortBuilder` builds `SerialPort` instances that connect to
USB serial devices, which are file
paths of the form `/dev/ttyUSBn`, where `n` is 0, 1, or 2. Note that
the `SerialPort` class is provided by the WPI library. 

### VacuousDisplayConnection

The `VacuousDisplayConnection` is a `DisplayConnection` implementation
that does absolutely nothing, which is used when a connection to a
marquee cannot be found. It allows application code to send messages
to a marquee when no physical connection exists.

## Commands

As described above, the [DisplayMessage](#displaymessage)
class is used to assemble and send a command to the marquee.
The patterns that the Marquee can display are specified in
the [DisplayCommand](#displaycommand) enumeration, and are
described below.

Note that times are in ESP32 Ticks, where a tick equals a millisecond.
Fields marked "N/A" are not used, and should not be set. Characters are
8 pixels high and 6 pixels wide, including white space.

Please see
[DisplayDriver.java](https://github.com/frc-team7587/Marquee-1/blob/main/Client/examples/sample/src/main/java/frc/robot/DisplayDriver.java)
for example commands.

### Error

The Marquee displays an error pattern when it receives an ill-formed
command (the usual case) or receives an explicit request to display
the error pattern. The error display consists of a barred rainbow ripple.
The explicit request is described immediately below.

Parameters:

| Parameter  | Contents               |
|------------|------------------------|
| Text       | N/A                    |
| Command    | `DisplayCommand.ERROR` |
| Foreground | N/A                    |
| Background | N/A                    |
| Delay 1    | N/A                    |
| Delay 2    | N/A                    |

### Single Color Fill

The color fill command sets every pixel on the panel to a
specified color. 

Parameters:

| Parameter  | Contents                         |
|------------|----------------------------------|
| Text       | N/A                              |
| Command    | `DisplayCommand.FILL_WITH_COLOR` |
| Foreground | Color to display                 |
| Background | N/A                              |
| Delay 1    | N/A                              |
| Delay 2    | N/A                              |

### Flash Two Colors

This command has not been implemented. Do not use it.

### Rippling Rainbow

Displays a rainbow pattern that ripples from left to right. New
colors enter from the left and disappear from the right. 

Parameters:

| Parameter  | Contents                                  |
|------------|-------------------------------------------|
| Text       | N/A                                       |
| Command    | `DisplayCommand.RIPPLING_RAINBOW`         |
| Foreground | N/A                                       |
| Background | N/A                                       |
| Delay 1    | Time between color shifts in milliseconds |
| Delay 2    | N/A                    |

### Single Pixel in Natural Order

Displays a simple test pattern, a single pixel that zigzags through
the display. The apparant path shows how the pixels are addressed.

Contrary to its appearance, the display is not a true array.
Rather, it is a line of LEDs that has been folded zigzag fashion
so it looks like an aray. At the very lowest level, pixels are
addressed linearly, indexed from 0 to 255.

The pattern cycles through a sequence of standard colors, changing
colors between traversals. 

Parameters:

| Parameter  | Contents                                    |
|------------|---------------------------------------------|
| Text       | N/A                                         |
| Command    | `DisplayCommand.SINGLE_PIXEL_NATURAL_ORDER` |
| Foreground | N/A                                         |
| Background | N/A                                         |
| Delay 1    | N/A                                         |
| Delay 2    | N/A                                         |

### Single Pixel in Left to Right, Bottom to Top Order

Displays a single pixel that moves left to right on the screen,
starting at the bottom row and moving upwards with each traversal.
The display cycles through a set of predefined colors, changing
color when the pattern restarts.

Parameters:

| Parameter  | Contents                                    |
|------------|---------------------------------------------|
| Text       | N/A                                         |
| Command    | `DisplayCommand.SINGLE_PIXEL_LEFT_TO_RIGHT` |
| Foreground | N/A                                         |
| Background | N/A                                         |
| Delay 1    | N/A                                         |
| Delay 2    | N/A                                         |

### Static Text

Displays up to five characters on the panel. Text is centered

Parameters:

| Parameter  | Contents                                    |
|------------|---------------------------------------------|
| Text       | Text to display, up to 5 characters         |
| Command    | `DisplayCommand.STATIC_TEXT`                |
| Foreground | Text (foreground) color                     |
| Background | Non-Text (background) color                 |
| Delay 1    | N/A                                         |
| Delay 2    | N/A                                         |

For example, to display "7587" in red on a black background, 
set the display text to `"7587"` and foreground red to 63.

### Text Scroll

The text scroll displays the start of the text, then shifts
the text to the left one column at a time. 

Parameters:

| Parameter  | Contents                                    |
|------------|---------------------------------------------|
| Text       | Text to display                             |
| Command    | `DisplayCommand.SCROLLING_TEXT`             |
| Foreground | Text (foreground) color                     |
| Background | Non-Text (background) color                 |
| Delay 1    | Time to display the initial text            |
| Delay 2    | Time between column shifts                  |

### Text Crawl

The text crawl shifts text in from the right one column at a
time. Text crawls smoothly to the left until its last character
vanishes completely, whereupon scroll starts again. 


Parameters:

| Parameter  | Contents                                    |
|------------|---------------------------------------------|
| Text       | Text to display                             |
| Command    | `DisplayCommand.TEXT_CRAWL`                 |
| Foreground | Text (foreground) color                     |
| Background | Non-Text (background) color                 |
| Delay 1    | Time between column shifts                  |
| Delay 2    | N/A                                         |


