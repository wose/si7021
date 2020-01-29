//! A platform agnostic driver to interface with the Si7021 humidity and
//! temperature sensor.
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

extern crate embedded_hal as hal;

use core::cmp;

use hal::blocking::i2c::{Read, Write, WriteRead};

const POLYNOMIAL: u32 = 0x13100;

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
    pub fn cmd(self) -> u8 {
        self as u8
    }
}

/// Errors
#[derive(Debug)]
pub enum Error<E> {
    /// Wrong CRC
    Crc,
    /// I2C bus error
    I2c(E),
}

/// Firmware Version
pub enum FirmwareVersion {
    /// Version 1.0
    V1_0,
    /// Version 2.0
    V2_0,
    /// Unknown Version
    Unknown(u8),
}

/// Heater Level
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum HeaterLevel {
    /// Typical Current Draw 3.090 mA @VDD = 3.3V
    L1 = 0b0000,
    /// Typical Current Draw 9.164 mA @VDD = 3.3V
    L2 = 0b0001,
    /// Typical Current Draw 15.238 mA @VDD = 3.3V
    L3 = 0b0010,
    /// Typical Current Draw 21.312 mA @VDD = 3.3V
    L4 = 0b0011,
    /// Typical Current Draw 27.386 mA @VDD = 3.3V
    L5 = 0b0100,
    /// Typical Current Draw 33.460 mA @VDD = 3.3V
    L6 = 0b0101,
    /// Typical Current Draw 39.534 mA @VDD = 3.3V
    L7 = 0b0110,
    /// Typical Current Draw 45.608 mA @VDD = 3.3V
    L8 = 0b0111,
    /// Typical Current Draw 51.682 mA @VDD = 3.3V
    L9 = 0b1000,
    /// Typical Current Draw 57.756 mA @VDD = 3.3V
    L10 = 0b1001,
    /// Typical Current Draw 63.830 mA @VDD = 3.3V
    L11 = 0b1010,
    /// Typical Current Draw 69.904 mA @VDD = 3.3V
    L12 = 0b1011,
    /// Typical Current Draw 75.978 mA @VDD = 3.3V
    L13 = 0b1100,
    /// Typical Current Draw 82.052 mA @VDD = 3.3V
    L14 = 0b1101,
    /// Typical Current Draw 88.126 mA @VDD = 3.3V
    L15 = 0b1110,
    /// Typical Current Draw 94.200 mA @VDD = 3.3V
    L16 = 0b1111,
}

impl HeaterLevel {
    /// Get heater control register value.
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// VDD Status
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum VddStatus {
    /// VDD is between 1.8V and 1.9V.
    /// If VDD drops below 1.8V, the device will no longer operate correctly.
    Low,
    /// VDD is above 1.9V.
    Ok,
}

/// Measurement Resolution
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum Resolution {
    /// RH: 12 bit, Temp: 14 bit
    RH12Temp14 = 0b0000_0000,
    /// RH: 8 bit, Temp: 12 bit
    RH8Temp12 = 0b0000_0001,
    /// RH: 10 bit, Temp: 13 bit
    RH10Temp13 = 0b1000_0000,
    /// RH: 11 bit, Temp: 11 bit
    RH11Temp11 = 0b1000_0001,
}

impl Resolution {
    /// Get register value.
    pub fn res(self) -> u8 {
        self as u8
    }
}

/// Si7021 Driver
pub struct Si7021<I2C> {
    i2c: I2C,
}

impl<I2C, E> Si7021<I2C>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{
    /// Creates a new driver from an I2C peripheral.
    pub fn new(i2c: I2C) -> Self {
        Si7021 { i2c }
    }

    /// Starts a humidity measurement and waits for it to finish before
    /// returning the measured value together with the temperature without
    /// doing a separate temperature measurement.
    pub fn humidity_temperature(&mut self) -> Result<(u16, i16), Error<E>> {
        let humidity = self.humidity()?;
        self.command(Command::ReadTempPostHumMeasurement)?;

        // temperature reading after humidity measurement doesn't support CRC
        let temperature = convert_temperature(self.read_u16()?);
        Ok((humidity, temperature))
    }

    /// Starts a humidity measurement and waits for it to finish before
    /// returning the measured value.
    pub fn humidity(&mut self) -> Result<u16, Error<E>> {
        self.command(Command::MeasureRelHumNoHoldMaster)?;

        // The total conversion time t_conv(RH) + t_conv(T) depends on the
        // sensor chip. As long as the conversion hasn't finished the sensor
        // will NAK the read.

        // This looks ugly... is there a better way to check for Ok or a
        // specific Err?
        loop {
            match self.read_u16_with_crc() {
                Err(Error::Crc) => return Err(Error::Crc),
                Err(_) => continue,
                Ok(rh_code) => return Ok(convert_humidity(rh_code)),
            }
        }
    }

    /// Starts a temperature measurement and waits for it to finish before
    /// returning the measured value.
    pub fn temperature(&mut self) -> Result<i16, Error<E>> {
        self.command(Command::MeasureTempNoHoldMaster)?;

        // The temperature conversion time t_conv(T) depends on the sensor
        // chip. As long as the conversion hasn't finished the sensor will NAK
        // the read.

        // This looks ugly... is there a better way to check for Ok or a
        // specific Err?
        loop {
            match self.read_u16_with_crc() {
                Err(Error::Crc) => return Err(Error::Crc),
                Err(_) => continue,
                Ok(temp_code) => return Ok(convert_temperature(temp_code)),
            }
        }
    }

    /// Issues a software reset.
    pub fn reset(&mut self) -> Result<(), Error<E>> {
        self.command(Command::Reset)?;
        Ok(())
    }

    /// Sets the measurement resolution.
    pub fn set_resolution(&mut self, res: Resolution) -> Result<(), E> {
        let reg = self.read_user_reg()? & 0x7E | res.res();
        self.i2c
            .write(ADDRESS, &[Command::WriteUserReg1.cmd(), reg])
    }

    /// Returns the current measurement resolution.
    pub fn get_resolution(&mut self) -> Result<Resolution, E> {
        let reg = self.read_user_reg()?;

        match reg & 0x81 {
            0x00 => Ok(Resolution::RH12Temp14),
            0x01 => Ok(Resolution::RH8Temp12),
            0x80 => Ok(Resolution::RH10Temp13),
            0x81 => Ok(Resolution::RH11Temp11),
            _ => unreachable!(),
        }
    }

    /// Sets the heater level.
    pub fn set_heater_level(&mut self, level: HeaterLevel) -> Result<(), E> {
        self.i2c
            .write(ADDRESS, &[Command::ReadHeaterCtrlReg.cmd(), level.value()])
    }

    /// Enables the heater.
    pub fn enable_heater(&mut self) -> Result<(), E> {
        self.control_heater(0x04)
    }

    /// Disables the heater.
    pub fn disable_heater(&mut self) -> Result<(), E> {
        self.control_heater(0x00)
    }

    fn control_heater(&mut self, enable: u8) -> Result<(), E> {
        let reg = self.read_user_reg()? & 0xFB | enable;
        self.i2c
            .write(ADDRESS, &[Command::WriteUserReg1.cmd(), reg])
    }

    /// Returns the VDD Status. If the operating voltage drops below 1.9 V, this
    /// will return `false`. If the operating voltage drops below 1.8 V, the device
    /// will no longer operate correctly.
    pub fn vdd_status(&mut self) -> Result<VddStatus, E> {
        let reg = self.read_user_reg()?;
        match reg & 0x40 {
            0 => Ok(VddStatus::Ok),
            _ => Ok(VddStatus::Low),
        }
    }

    fn read_user_reg(&mut self) -> Result<u8, E> {
        let mut buffer = [0];
        self.i2c
            .write_read(ADDRESS, &[Command::ReadUserReg1.cmd()], &mut buffer)?;
        Ok(buffer[0])
    }

    /// Reads the 64bit serial number and returns it as 8 bytes.
    ///
    /// - `buf[0]`: SNA_3
    /// - `buf[1]`: SNA_2
    /// - ...
    /// - `buf[7]`: SNB_0
    ///
    /// SNB_3 (`buf[4]`) contains the sensor id:
    ///
    /// - `0x00` or `0xFF`: engineering samples
    /// - `0x0D = 13`: Si7013
    /// - `0x14 = 20`: Si7020
    /// - `0x15 = 21`: Si7021
    /// - `0x32 = 50`: HTU21D/SHT21
    pub fn serial(&mut self) -> Result<[u8; 8], Error<E>> {
        let mut serial = [0u8; 8];
        let mut buffer = [0u8; 8];
        self.i2c.write(ADDRESS, &[0xFA, 0x0F]).map_err(Error::I2c)?;
        self.i2c.read(ADDRESS, &mut buffer).map_err(Error::I2c)?;
        serial[0] = buffer[0];
        serial[1] = buffer[2];
        serial[2] = buffer[4];
        serial[3] = buffer[6];

        let mut buffer = [0u8; 6];
        self.i2c.write(ADDRESS, &[0xFC, 0xC9]).map_err(Error::I2c)?;
        self.i2c.read(ADDRESS, &mut buffer).map_err(Error::I2c)?;
        serial[4] = buffer[0];
        serial[5] = buffer[1];
        serial[6] = buffer[3];
        serial[7] = buffer[4];
        self.check_crc(&buffer[0..2], buffer[2])?;
        // self.check_crc(&buffer[3..5], buffer[5])?;
        Ok(serial)
    }

    /// Reads the firmware revision.
    ///
    /// - `0xFF`: Firmware version 1.0
    /// - `0x20`: Firmware version 2.0
    pub fn firmware_rev(&mut self) -> Result<FirmwareVersion, E> {
        let mut buffer = [0];
        self.i2c.write(ADDRESS, &[0x84, 0xB8])?;
        self.i2c.read(ADDRESS, &mut buffer)?;
        match buffer[0] {
            0xFF => Ok(FirmwareVersion::V1_0),
            0x20 => Ok(FirmwareVersion::V2_0),
            ver => Ok(FirmwareVersion::Unknown(ver)),
        }
    }

    fn read_u16(&mut self) -> Result<u16, Error<E>> {
        let mut buffer = [0, 0];
        self.i2c.read(ADDRESS, &mut buffer).map_err(Error::I2c)?;
        Ok(((buffer[0] as u16) << 8) + (buffer[1] as u16))
    }

    fn read_u16_with_crc(&mut self) -> Result<u16, Error<E>> {
        let mut buffer = [0u8; 3];
        self.i2c.read(ADDRESS, &mut buffer).map_err(Error::I2c)?;
        self.check_crc(&buffer[0..2], buffer[2])?;
        Ok(((buffer[0] as u16) << 8) + (buffer[1] as u16))
    }

    fn command(&mut self, command: Command) -> Result<(), Error<E>> {
        self.i2c
            .write(ADDRESS, &[command.cmd()])
            .map_err(Error::I2c)
    }

    fn check_crc(&self, data: &[u8], crc: u8) -> Result<(), Error<E>> {
        let mut data = (((data[0] as u16) << 8) + (data[1] as u16)) as u32;
        for _ in 0..16 {
            if data & 0x8000 != 0 {
                data = (data << 1) ^ POLYNOMIAL;
            } else {
                data <<= 1;
            }
        }

        data >>= 8;
        if data == crc as u32 {
            Ok(())
        } else {
            Err(Error::Crc)
        }
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
