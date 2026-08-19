use embassy_futures::join::join;
use embassy_rp::{
    Peri, bind_interrupts,
    peripherals::USB,
    usb::{Driver, InterruptHandler},
};
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    {Builder, Config},
};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::task]
pub async fn usb_setup(p: Peri<'static, USB>) {
    let driver = Driver::new(p, Irqs);

    let mut config = Config::new(0xbaba, 0xdead);
    config.manufacturer = Some("Embassy");
    config.product = Some("ICM20948 Dump");
    config.serial_number = Some("0xBABADEAD");
    config.max_power = 250;
    config.max_packet_size_0 = 64;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut logger_state = State::new();
    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );
    let logger_class = CdcAcmClass::new(&mut builder, &mut logger_state, 64);
    let log_fut = embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, logger_class);
    let mut usb = builder.build();
    let usb_fut = usb.run();

    join(usb_fut, log_fut).await;
}
