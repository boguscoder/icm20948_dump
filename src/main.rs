#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    i2c::{self, Config, InterruptHandler as I2CInterruptHandler},
    peripherals::I2C0,
};
use embassy_time::{Delay, Duration, Ticker};
use icm20948_async::*;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2CInterruptHandler<I2C0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    spawner.spawn(usb_console::usb_setup(p.USB, Default::default()).unwrap());

    // IMU via i2c
    let sda = p.PIN_12;
    let scl = p.PIN_13;

    log::info!("set up i2c ");
    let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, Config::default());

    let imu_result = Icm20948::new_i2c_from_cfg(
        i2c,
        icm20948_async::Icm20948Config {
            acc_range: AccRange::Gs8,
            gyr_range: GyrRange::Dps2000,
            acc_unit: AccUnit::Mpss,
            gyr_unit: GyrUnit::Rps,
            acc_dlp: AccDlp::Hz111,
            gyr_dlp: GyrDlp::Hz361,
            acc_odr: 0,
            gyr_odr: 0,
        },
        Delay,
    )
    .set_address(0x69)
    .initialize_9dof()
    .await;

    let Ok(mut imu) = imu_result else {
        panic!("Failed to initialize IMU")
    };

    let mut imu_ticker = Ticker::every(Duration::from_hz(1000));

    log::info!("Format Acc(vec3) Gyr(vec3) Mag(vec3)");
    loop {
        let Ok(measurement) = imu.read_9dof().await else {
            continue;
        };

        log::info!(
            "{}, {}, {}, {}, {}, {}, {}, {}, {}",
            measurement.acc.x,
            measurement.acc.y,
            measurement.acc.z,
            measurement.gyr.x,
            measurement.gyr.y,
            measurement.gyr.z,
            measurement.mag.x,
            measurement.mag.y,
            measurement.mag.z
        );
        // Delay until next loop
        imu_ticker.next().await;
    }
}
