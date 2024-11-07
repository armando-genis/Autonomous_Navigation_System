# Encoders

Configuration scripts and ROS2 node drivers for the encoders:

* 1 x [IFM RM8004](https://www.ifm.com/mx/es/product/RM8004?tab=documents) - Absolute Multiturn Encoder
    * 4096 revolutions, 24-bit resolution
* 2 x [Briter](https://briterencoder.com/product/canbus-multi-turn-absolute-rotary-encoder/) CANbus Multi-turn Absolute Rotary Encoder
    * 24 revolutions, 12-bit resolution

# IMPORTANT!!

Set encoder return times to 100 Hz for control purposes !!!!!
If canbus is filled, try reducing up to 20 Hz, no less !!

## IFM RM8004

This encoder has a preoperational and operational mode. The preoperational mode is used to load and save parameters to the encoder by
sending CAN frames to the device. Once the configuration is complete, the operational mode must be enabled to actually use the device.

The encoder node number is 1Fh + 1h = 20h.

### Pre-operational mode
On connection to the supply, the encoder automatically enters to the preoperational mode, which is indicated by the LED in green (no flashing).

Here is a frame template to read/write objects to the encoder:

``` 
cansend can0 ID # COMMAND INDEX SUBID DATA

```

ID: 600h + NODE NUMBER
COMMAND: (1 byte)
INDEX: (2 bytes, big endian) object index
SUBID: (1 byte)
DATA: (4 bytes, big endian)

The COMMAND byte is chosen accordingly to a read/write operation. Check the next table:
| COMMAND | DATA LENGTH | DATA TYPE | FUNCTION |
|---------|-------------|-----------|----------|
| 43h     | 4           | u32       | read     |
| 47h     | 3           | u24       | read     |
| 4Bh     | 2           | u16       | read     |
| 4Fh     | 1           | u8        | read     |
| 23h     | 4           | u32       | write    |
| 27h     | 3           | u24       | write    |
| 2Bh     | 2           | u16       | write    |
| 2Fh     | 1           | u8        | write    |

The object, index, sub index, and data depends on the what is required. For example, to check the current encoder position, one must send (check datasheet for details):

``` 
cansend can0 620#43046000

``` 
And to store all parameters:

``` 
cansend can0 620#2310100173617665

``` 
Check datasheet on how to change baud rates or the cyclic timer.

To change the preset value (change zero position of the encoder) use (store after using this command):

``` 
cansend can0 620#2303600000000000

``` 

### Operational mode
To enter the operational mode, the 2 bytes must be send 01h 00h, with the 00h identifier. If using socketCAN through the terminal, the next command
must be used:

``` 
cansend can0 000#0100 

```

Once the operational mode is entered, the LED will start flashing each second.
To go back to the pre-operational mode, the next command must be used:

``` 
cansend can0 000#8000 

```

In the operational mode the first transmit PDO (1A0h) is used to send the current position value without any request by the host. The cycle time can be programmed in milliseconds for values between 1 ms and 65536 ms (this must be configured in the pre-operational mode).

To set cyclic time to 100ms (check datasheet for details):

``` 
cansend can0 620#2B00620064000000

```
The encoder is currently configured to send data at 20Hz

Use `candump -td can0` to check the canbus and the difference in time between messages to verify.

Don't forget to store parameters.

## Briter

This encoder is a little bit less sophisticated, it provides no modes, rather you are supposed to configure everything "on the fly". It has internal memory so it can save previous configurations.

Having a resolution of 12 bits, each revolution consists of 4096 steps, with a total of 4096*24 = 98304 steps.

For the encoder to understand a command, you need to follow this structure:

| Data Length | Encoder Address | Command | Data      |
|-------------|-----------------|---------|-----------|
| 1 Byte      | 1 Byte          | 1 Byte  | 0~4 Bytes |

(Data length includes it's own field in the calculated length)
(The Data column, is encoded in little endian)

### Configuration commands
**Set the encoder's ID - 0x02**
```
Send command to 0x01, to change it's id to 0x02
[0x04, 0x01, 0x02, 0x02]
```

**Set the encoder's baud rate - 0x03**
0x00:500K; 0x01:1M 0x02:250K; 0x03:125K; 0x04:100K;
```
Send command to 0x01, to set the baud rate of 100,000
[0x04, 0x01, 0x03, 0x04]
```

**Set the encoder's mode - 0x04**
0x00: query; 0xAA: automatic post back;
```
Send command to 0x01, change its mode to autmoatic
[0x04, 0x01, 0x04, 0xAA]
```

**Set the encoder's return time - 0x05**
```
Only valid in automatic mode. Time provided in microseconds
Send command to 0x01, change the interval to 1000us
[0x05, 0x01, 0x05, 0xE8, 0x03]
```
Or to set it to 10000us (10ms)
``` 
cansend can0 001#0501051027

```

### Operation commands
**Read the encoder value - 0x01**
This command is really only useful if using the query mode, automatic mode with return this without asking.
```
Send command to 0x01, to read the encoder's position
[0x04, 0x01, 0x01, 0x00]
```

**Set current position to 0 - 0x06**
Just follow the datasheet and connect the yellow wire to GND for more than 100ms. Encoder should not be operational during this!! Not in query mode, nor commuicating. To be shure just power off and turn on again, and then connect to GND.

The other option:

```
Send command to 0x01, to set the position to 0
[0x04, 0x01, 0x06, 0x00]
```

**Set an exact position on the encoder - 0x0D**
```
Send command to 0x01, to change it's position to 74565
[0x07, 0x01, 0x0D, 0x00, 0x01, 0x23, 0x45]
```

### How to interpret the encoder's position?
Here are two code snippets, because for some reason, the encoder returns the data in two diferent formats, depending if it is on automatic or query mode.

**automatic mode**
```
int( ((receivedMsg.data.hex()[6:])[0:4])[::-1], 16)
```

**Usado para leer del query mode**
```
int( (receivedMsg.data.hex()[6:])[::-1], 16)
```

^^ Both of the codes basically, just reverse it converting it from low-endian to big-endian, remove the first 3 bytes (which is metadata), and convert it into an integer with the built-in python function.