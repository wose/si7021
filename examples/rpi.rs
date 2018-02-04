extern crate linux_embedded_hal as hal;
extern crate si7021;

use std::thread;
use std::time::Duration;

use hal::{Delay, I2cdev};
use si7021::{Resolution, Si7021, FirmwareVersion};

fn main() {
    let dev = I2cdev::new("/dev/i2c-1", si7021::ADDRESS as u16).unwrap();
    let mut si7021 = Si7021::new(dev, Delay);

    print!("SN: ");
    for byte in &si7021.serial().unwrap() {
        print!("{:02X} ", byte);
    }

    println!();
    print!("Firmware Version: ");
    match si7021.firmware_rev().unwrap() {
        FirmwareVersion::V1_0 => println!("V1 (0xFF)"),
        FirmwareVersion::V2_0 => println!("V2 (0x20)"),
        FirmwareVersion::Unknown(ver) => println!("Unknown ({})", ver),
    }

    for _ in 0..10 {
        let (humidity, temperature) = si7021.humidity_temperature().unwrap();
        println!("{:>6.2}% {:6.2}°C", humidity as f32 / 100.0, temperature as f32 / 100.0);
        thread::sleep(Duration::from_secs(1));
    }

    println!("Set new Resolution");
    si7021.set_resolution(Resolution::RH8Temp12).unwrap();
    println!("{:?}", si7021.get_resolution().unwrap());

    println!("Reset");
    si7021.reset().unwrap();

    println!("Resolution after Reset");
    println!("{:?}", si7021.get_resolution().unwrap());

    for _ in 0..10 {
        let (humidity, temperature) = si7021.humidity_temperature().unwrap();
        println!("{:>6.2}% {:>6.2}°C", humidity as f32 / 100.0, temperature as f32 / 100.0);
        thread::sleep(Duration::from_secs(1));
    }
}
