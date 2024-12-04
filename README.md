# SignalK-SensESP-Boat-Systems-Monitor

## Intro
This project is my attempt at creating a boat systems' monitoring device for use with a Signal K server so that the data can be presented on dashboard displays as seen fit.

The main board is a well-known board from Hat Labs called... TODO: What is it called? LOL

This project uses two small add-on boards from Hat labs to provide four ADC channels each on the I2C bus utilizing the ADS1115 IC. A custom PCB was made to provide four channels of 4-20mA inputs with simple and reasonable input protection to the ADC boards' inputs. This gives the project eight 4-20mA input channels protected by a TVS diode, shunting diodes, inline resistance and a 50mA Polyfuse per channel. See the schematic here of one of the channel input protections:

![Schematic diagram of one of the 4-2mA input processing and protection circuits for the ADC channels](docs/images/4-20mA_schematic_one_channel.png?raw=false)
Precarious_test_setup_IMG_1652

The left-side of the schematic shows the screw terminals to connect the 4-20mA field wiring to. The 165 ohm resistor converts the 4-20mA signal into a 0.66v to 3.3v signal to the ADC input pin. The BAT54S IC has two Schottkey diodes to clamp any over/under voltage to ground or 3.3v rail to protect the ADS1115 ADC input. There is also a 50mA self-resetting fuse (shown as a resistor in the schematic :eyeroll:).

## What Does It Sense?
The Systems Monitor device (or SysMon for short) will be responsible for reading all the boat data needed to monitor and keep the systems operating as they should. Tank levels will be known, engine running parameters will be known, etc. All of this data will be available on dashboards and small displays as needed while moored or underway.

Here is a list of the proposed data points gathered and recorded by the SysMon device:

#### General/Environment
--------------------
 * outdoor temperature
 * Raw water temperature
 * indoor temperature
 * indoor luminosity (CYD/LDR)
 * indoor humidity
 
#### Systems/Domestic
--------------------
 * Fresh water tank level
 * Fresh water tank temperature
 * Cooler/fridge/freezer temperature
 
#### Systems/Engine
--------------------
 * Raw water temperature
 * Engine coolant temperature
 * Diesel tank level
 * Engine oil pressure
 * bilge pump frequency, status indicator, history plot?
 * Battery voltage
 * Battery current
 * Alternator charge current (CSLF5HE)

## Debug Display
The SH-ESP32 board has a convienient I2C connector that can be used for a small OLED display sold by Hat Labs. This little display has been included in this project to show debug data and raw data readings for testing and calibration. 

The display has a very basic organization whereas the screen of 128 x 64 pixels is divided into four columns and eight rows of text "cells". This should allow enough space to show most/all of the raw data points as well as some status messages.

The font size of 1 has characters of 6 x 8 pixels with a one pixel baseline spacing. Seen below is an image of the text cells and some text characters for comparison.

![Diagram grid of the display layout with soe text boudries](src/displays/SSD1306Display.svg?raw=true)


Here is a photo of a very rough and somewhat precarious test setup to make sure everything is working as it should. This was my first actual test after writing most of the code blindly. 

![Photo of first trial test setup](docs/images/Precarious_test_setup_IMG_1652.JPG?raw=false)

This test setup has a single 1-wire temperature probe, a 4-20mA connected 0-100C RTD temperature transmitter, battery voltage and current and the OPTO_IN bilge pump status input. All worked perfectly except the OLED display didn't. I had gotten an SPI version of the SSD1306 display and modified it for I2C, but it wouldn't go. I have since ordered a proper replacement from Hat Labs.

## Specialized Hardware And Sensors
As part of my experimentation with all of this, I ended up using some pretty expensive industrial signal conditioner devices to monitor the batteries with. I wanted something with isolation and with 4-20mA outputs for ease of use with my system. The goal in the future will be to replace these devices with dedicated hardware interfaces that don't cost so much and aren't as bulky.

The devices I have were bought much cheaper used on Ebay. They only required a small amount of refurbishing and setting so DIP switches for my application.

### FC-33 Signal Conditioner for reading battery voltage
[FC-33 Datasheet](https://cdn.automationdirect.com/static/specs/fcsignalconditioners.pdf)


### FC-B34 Bipolar Signal Conditioner for reading battery current shunt
[FC-B34 Datasheet](https://cdn.automationdirect.com/static/specs/fcbiposigcond.pdf)

