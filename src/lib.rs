//! A platform agnostic driver to interface with the Si7021 humidity and
//! temperature sensor.
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.1

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

extern crate embedded_hal as hal;

use core::cmp;

use hal::blocking::delay::DelayMs;
use hal::blocking::i2c::{Read, Write, WriteRead};

/// I2C address
pub const ADDRESS: u8 = 0x40;

#[allow(dead_code)]
#[derive(Copy, Clone)]
enum Command {
    MeasureRelHumHoldMaster = 0xE5,
    MeasureRelHumNoHoldMaster = 0xF5,
    MeasureTempHoldMaster = 0xE3,
    MeasureTempNoHoldMaster = 0xF3,
    ReadTempPostHumMeasurement = 0xE0,
    Reset = 0xFE,
    WriteUserReg1 = 0xE6,
    ReadUserReg1 = 0xE7,
    WriteHeaterCtrlReg = 0x51,
    ReadHeaterCtrlReg = 0x11,
}

impl Command {
    pub fn cmd(&self) -> u8 {
        *self as u8
    }
}

/// Si7021 Driver
pub struct Si7021<I2C, D> {
    i2c: I2C,
    delay: D,
}

impl <I2C, D, E> Si7021<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u8>,
{
    /// Creates a new driver from an I2C peripheral.
    pub fn new(i2c: I2C, delay: D) -> Self {
        Si7021 { i2c, delay }
    }

    /// Starts a humidity measurement and waits for it to finish before
    /// returning the measured value together with the temperature without
    /// doing a separate temperature measurement.
    pub fn humidity_temperature(&mut self) -> Result<(u16, i16), E> {
        let humidity = self.humidity()?;
        self.i2c.write(ADDRESS, &[Command::ReadTempPostHumMeasurement.cmd()])?;
        let temperature = convert_temperature(self.read_u16()?);
        Ok((humidity, temperature))
    }

    /// Starts a humidity measurement and waits for it to finish before
    /// returning the measured value.
    pub fn humidity(&mut self) -> Result<u16, E> {
        self.i2c.write(ADDRESS, &[Command::MeasureRelHumNoHoldMaster.cmd()])?;

        // wait for total conversion time t_conv(RH) + t_conv(T)
        // max(t_conv(RH)) = 12ms
        // max(t_conv(T)) = 10.8ms
        self.delay.delay_ms(23);

        let rh_code = self.read_u16()?;
        Ok(convert_humidity(rh_code))
    }

    /// Starts a temperature measurement and waits for it to finish before
    /// returning the measured value.
    pub fn temperature(&mut self) -> Result<i16, E> {
        self.i2c.write(ADDRESS, &[Command::MeasureTempNoHoldMaster.cmd()])?;

        // wait for temperature conversion time t_conv(T)
        // max(t_conv(T)) = 10.8ms
        self.delay.delay_ms(11);

        let temp_code = self.read_u16()?;
        Ok(convert_temperature(temp_code))
    }

    fn read_u16(&mut self) -> Result<u16, E> {
        let mut buffer = [0, 0];
        self.i2c.read(ADDRESS, &mut buffer)?;
        Ok(((buffer[0] as u16) << 8) + (buffer[1] as u16))
    }
}

fn convert_humidity(rh_code: u16) -> u16 {
    let humidity = ((12500 * rh_code as u32) >> 16) as i16 - 600;

    // sensor may report slightly more than 100% or less than 0 so we clamp it
    cmp::min(cmp::max(humidity, 0), 10000) as u16
}

fn convert_temperature(temp_code: u16) -> i16 {
    ((17572 * temp_code as u32) >> 16) as i16 - 4685
}
