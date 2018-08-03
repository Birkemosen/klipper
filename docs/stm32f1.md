This document describes how the STM32F1 port operates and how it can be used on
STM32-based boards, such as the "Blue Pill". STM32 MCUs are not used on any
Arduino boards, so their restrictions aren't as widely known and less straight
forward compared to common Arduino compatible boards. There aren't any standard
pin mappings either.

General considerations
======================

The STM32 port currently requires an 8 MHz crystal for correct operation. The
host communication only supports a standard UART. USB serial (CDC ACM) might be
useful for some configurations, but is unimplemented. The port is currently
designed for and tested with STM32F103xB series MCUs, but it should work with
any STM32F103 series MCUs with minimal changes.

Unlike Arduino-based boards, typically there is no automatic reset on serial
connection with STM32 boards. Please use `restart_method: command` with the
STM32F1 port.

Fixed pins
==========

The UART used for communication with the host is fixed to pins PA9 (TX) and PA10
(RX). SWD pins (PA13/PA14) are enabled for debugging and cannot be used for any
I/O. SPI uses pins PB13/PB14/PB15, but the pins can be used as general I/O if
SPI is not used.

Digital I/O
===========

All pins that aren't part of the fixed set can be used for digital I/O. Pins are
referred to by their primary name, e.g. `PA1`. Do not try to use Arduino pin
aliases in your configuration. See ST's datasheets for more details. The
[STM32Duino](http://wiki.stm32duino.com/index.php?title=Blue_Pill) wiki has more
info on the popular "Blue Pill" board.

Analog inputs
=============

All ADC-capable pins can be used as analog inputs with the same naming as
digital I/O pins. Small packages MCUs (e.g. LFQP48) have 10 channels (PA0-PA7,
PB0-PB1), while larger package devices have 16 channels (PA0-PA7, PB0-PB1,
PC0-PC5).

SPI
===

SPI uses pin PB13 (SCK), PB14 (MISO) and PB15 (MOSI). The clock speed range is
0.15..18 MHz. Chip select pins do not have any restrictions.
