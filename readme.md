# CAN 2.0B (Molinaro): Controller Area Network (CAN) Analyzer for Saleae logic Analyzers

This is a plugin for Saleae logic Analyzers, built with Saleae Analyzer SDK.

## Building the plugin

For building the plugin, see [https://support.saleae.com/saleae-api-and-sdk/protocol-analyzer-sdk/build] (https://support.saleae.com/saleae-api-and-sdk/protocol-analyzer-sdk/build) 

For installing the plugin, see [https://support.saleae.com/faq/technical-faq/setting-up-developer-directory](https://support.saleae.com/faq/technical-faq/setting-up-developer-directory)

## Selecting Analyzer

The analyzer Name is `CAN 2.0B (Molinaro)`.

![Setting Dialog](readme-images/selecting-analyzer.png)

## Plugin settings


![Setting Dialog](readme-images/setting-dialog.png)

### CAN Bit Rate

Usual CAN bit rates settings are `1000000` (1 Mbit/s), `500000` (500 kbit/s), `250000` (250 kbit/s), `125000` (125 kbit/s), `62500` (62.5 kbit/s). But you can use any custom setting (maximum is 1 Mbit/s).

### Dominant Logic Level

Usually, CAN Dominant level is `LOW` logic level. This setting enables selecting `HIGH` as dominant level. 

### Simulator ACK SLOT generated level

*This setting is only used be the simulator. The simulator is enabled when no device is connected the analyzer.*

The `ACK SLOT` field of a CAN 2.0B frame is sent recessive, and set dominant by any receiver that gets the frame without any error.

Three settings are available:

* `Dominant`: the simulator generates frames with the ACK SLOT bit dominant;
* `Recessive`: the simulator generates frames with the ACK SLOT bit recessive;
* `Random`: the simulator generates frames with the ACK SLOT bit randomly dominant or recessive.

### Simulator Generated Frames

*This setting is only used be the simulator. The simulator is enabled when no device is connected the analyzer.*

You can set the format (standard / extended) and the the type (data / remote) that the simulator generates:

* `All Types`: the simulator randomly generates standard / extended, data / remote frames;
* `Only Standard Data Frames`: the simulator randomly generates standard data frames;
* `Only Extended Data Frames`: the simulator randomly generates extended data frames;
* `Only Standard Remote Frames`: the simulator randomly generates standard remote frames;
* `Only Extended Remote Frames`: the simulator randomly generates extended remote frames.

##Capture Display

This is the capture of a Standard data frame, identifier `0x7B1`, one data byte (`0xF1`), with `ACK SLOT` dominant.

![](readme-images/capture-left.png)
![](readme-images/capture-center.png)
![](readme-images/capture-right.png)

By default, the center of a bit is indicated with a white dot.

The green dot is the `SOF` (*Start Of Frame*) field.

A white `X` is a Stuff Bit.

A fixed form bit is denoted by a `0` (for a dominant bit) or a `1` (recessive bit). *Note the level inversion does not change the annotation.* 

The `RTR` and the `SRR` bits are denoted by an `up arrow` (if recessive), or a `down arrow` (if dominant).

Errors are in red color: a red `X` is a Stuff Error, and following bits are tagget with red dots until the bus returns free (11 consecutive recessive bits).

## Bubble Text

The bubble text is the text over the capture.

All CAN frames fields are reported, but:

* the `SOF` field that is denoted by a green dot.
* the `CRC DEL` field, a fixed form recessive bit after the `CRC` field.

If a CRC error is detected, the text is `CRC: xxx (error)`.
 
## Tabular Text

For each frame, the tabular text contains:

* the identifier;
* the data bytes;
* if there is a CRC error, the CRC field;
* the frame bit length, its duration, and the number of stuff bits.

![](readme-images/tabular-text.png)

