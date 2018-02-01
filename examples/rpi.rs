extern crate linux_embedded_hal as hal;
extern crate si7021;

use std::thread;
use std::time::Duration;

use hal::{Delay, I2cdev};
use si7021::Si7021;

fn main() {
    let dev = I2cdev::new("/dev/i2c-1", si7021::ADDRESS as u16).unwrap();
    let mut si7021 = Si7021::new(dev, Delay);

    for _ in 0..100 {
        let (humidity, temperature) = si7021.humidity_temperature().unwrap();
        println!("{:>6.2}% {:6.2}°C", humidity as f32 / 100.0, temperature as f32 / 100.0);
        thread::sleep(Duration::from_secs(1));
    }
}
