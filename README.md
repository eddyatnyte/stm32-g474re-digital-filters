# STM32G474RE – Real-Time DSP Filter Labs

This repository contains three STM32-based lab projects developed during my Master's program in Embedded Systems. Each project focuses on the real-time implementation of different digital filter types on the NUCLEO-G474RE development board using STM32CubeIDE.

## Project Structure

### 01_EMA-Filter
Implementation of low-pass and high-pass Exponential Moving Average (EMA) filters using floating-point arithmetic. Both filtered and unfiltered signals are output via DAC for oscilloscope analysis.

### 02_Resonator_Oscillator
Includes a second-order IIR resonator filter and a digital oscillator based on complex-conjugate poles. Implemented using floating-point arithmetic, demonstrating real-time synthesis and frequency shaping.

### 03_FIR-Q1.15
High-order FIR filter implementation using Q1.15 fixed-point arithmetic. Includes ADC input, DAC output, and execution time measurement. Focuses on efficiency and performance comparison to floating-point versions.

## Objectives

- Implement and analyze digital filters in real time using STM32CubeIDE
- Use ADC, DAC, TIM6 interrupts, and ISR-based processing
- Compare floating-point and fixed-point implementations
- Validate filter behavior using an oscilloscope

## Tools and Platform

- STM32CubeIDE
- NUCLEO-G474RE development board (STM32G474RE MCU, ARM Cortex-M4)
- Oscilloscope (e.g. PicoScope) for signal observation
- MATLAB for filter design and simulation

## Context

These lab projects were developed as part of the *Embedded Signal Processing Systems* course (Prof. Dr. C. Jakob) at Hochschule Darmstadt (h_da), Germany. They form part of the International Master's in Electrical Engineering program.

