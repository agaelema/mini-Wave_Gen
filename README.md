# mini Wave_Gen
Project of a mini wave generator using the DAC MCP4725


![alt text](https://github.com/agaelema/mini-Wave_Gen/blob/master/v01%20-%20Arduino%20nano/images/IMG_20170603_132831609_compressed.jpg?raw=true "mini Wave_Gen")

---

## Arduino nano version

This project presents a simple waveform generator based in an Arduino Nano (or Uno) and one DAC MCP4725.

There are some limitations related to sample rate due the use of Arduino Nano, but the result is very interesting.

### Basic components

Basic components
- Arduino Nano (Uno)
- DAC MCP4725 (i2c)
- OLED Display (i2c)
- Rotary encoder
- Switch
- Some passive components

### Circuit

As can be seen in the picture, the circuit is very simple with just some components.

![alt text](https://github.com/agaelema/mini-Wave_Gen/blob/master/v01%20-%20Arduino%20nano/images/mini-Wave_Gen_bb_edit_3.png?raw=true "Circuit of mini Wave_Gen")

### Waveforms

The mini Wave_Gen has 4 different waveforms
- Sine
- Ramp
- Square
- DC level

![alt text](https://github.com/agaelema/mini-Wave_Gen/blob/master/v01%20-%20Arduino%20nano/images/dac-sine-ramp-square-orig.png?raw=true "Waveforms")

Changing the sample rate is possible to balance between signal frequency and "perfection" of waveform. See the difference between 16, 32, 64 and 128 samples per cycle.

![alt text](https://github.com/agaelema/mini-Wave_Gen/blob/master/v01%20-%20Arduino%20nano/images/dac-sine-samples-orig.png?raw=true "Differente number os sample/cycle")

___


### DISCLAIMER

This software is provided 'as is' with no explicit or implied warranties in respect of its properties, including, but not limited to, correctness and/or fitness for purpose.
