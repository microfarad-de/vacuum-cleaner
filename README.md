# Brushless Vacuum Cleaner

Arduino firmware for a brushless vacuum cleaner controller, runs on an Arduino Pro Mini.

The purpose is a brushless motor upgrade for the "Balck&Decker DustBuster Flexi"
vacuum cleaner.

This sketch controls the Brushless ESC by simulating the behavior 
of a RC receiver. 

The throttle is set to minimum for a preset amount of time to 
allow the ESC to arm. The power is then slowly ramped-up until 
reaching the target operating power setting.

This sketch also controls a MOSFET module that keeps the ESC armed
for a preset amount of time (5 min) after the power switch has been
toggled to off.

For reference:

* Original brushed motor with impeller attached and vacuum chamber removed runs at 28990 RPM / 5 Amperes. 
  Loading the motor would result in increased current draw.

* Hobby Wing ESC is configured to run at 30000 RPM / 6 Amperes. 
  No current increase when loading the motor, the ESC keeps the current flow constant at all loads.

Unless stated otherwise within the source file headers, please feel free to use and distribute this code under the *GNU General Public License v3.0*.

## Prerequisites

* ATmega328P based Arduino Pro Mini, Arduino Nano or similar model
* Custom bootloader from: https://github.com/microfarad-de/bootloader

## Circuit Diagram

The circuit diagram for this device can be found under the */doc* folder or can be downloaded using the follwoing link:
https://github.com/microfarad-de/vacuum-cleaner/raw/master/doc/circuit-diagram.pdf
