#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod xinput;

use core::pin::pin;

use ::pio::{Instruction, InstructionOperands, MovDestination, MovOperation, MovSource};
use defmt::*;
use embassy_futures::join::join;
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::pio::{self, Direction, IrqFlags, Pio, ShiftConfig, ShiftDirection, StateMachine};
use embassy_rp::{bind_interrupts, i2c, peripherals, usb, Peripheral};
use embassy_time::{with_timeout, Duration, Ticker};
use fixed::traits::ToFixed;
use fixed_macro::types::U56F8;
use wii_ext::classic_async;
use wii_ext::core::classic::ClassicReadingCalibrated;
use xinput::{State, XInput};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<peripherals::USB>;
    PIO0_IRQ_0 => pio::InterruptHandler<peripherals::PIO0>;
    I2C0_IRQ => i2c::InterruptHandler<peripherals::I2C0>;
});

const CONTROLLER_STATE_INIT: State = State::new();
static CONTROLLER_STATE: [State; 4] = [CONTROLLER_STATE_INIT; 4];

fn wiiext_to_xinput(ext: ClassicReadingCalibrated) -> [u8; 12] {
    let mut xinput_data = [0_u8; 12];
    // ext.
    // xinput_data[0] |= map_bit(ext.)
    xinput_data[0] |= if ext.dpad_up { 0b1 } else { 0b0 } << 0;
    xinput_data[0] |= if ext.dpad_down { 0b1 } else { 0b0 } << 1;
    xinput_data[0] |= if ext.dpad_left { 0b1 } else { 0b0 } << 2;
    xinput_data[0] |= if ext.dpad_right { 0b1 } else { 0b0 } << 3;
    xinput_data[0] |= if ext.button_plus { 0b1 } else { 0b0 } << 4;
    xinput_data[0] |= if ext.button_minus { 0b1 } else { 0b0 } << 5;
    xinput_data[0] |= if ext.button_zl { 0b1 } else { 0b0 } << 6;
    xinput_data[0] |= if ext.button_zr { 0b1 } else { 0b0 } << 7;

    xinput_data[1] |= if ext.button_trigger_l { 0b1 } else { 0b0 } << 0;
    xinput_data[1] |= if ext.button_trigger_r { 0b1 } else { 0b0 } << 1;
    xinput_data[1] |= if ext.button_home { 0b1 } else { 0b0 } << 2;
    xinput_data[1] |= if ext.button_b { 0b1 } else { 0b0 } << 4;
    xinput_data[1] |= if ext.button_a { 0b1 } else { 0b0 } << 5;
    xinput_data[1] |= if ext.button_y { 0b1 } else { 0b0 } << 6;
    xinput_data[1] |= if ext.button_x { 0b1 } else { 0b0 } << 7;

    xinput_data[2] = ext.trigger_left as u8; // L2
    xinput_data[3] = ext.trigger_right as u8; // R2

    [xinput_data[4], xinput_data[5]] = [ext.joystick_left_x as u8, ext.joystick_left_x as u8];
    [xinput_data[6], xinput_data[7]] = [ext.joystick_left_y as u8, ext.joystick_left_y as u8];
    [xinput_data[8], xinput_data[9]] = [ext.joystick_right_x as u8, ext.joystick_right_x as u8];
    [xinput_data[10], xinput_data[11]] = [ext.joystick_right_y as u8, ext.joystick_right_y as u8];
    xinput_data
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut p = embassy_rp::init(Default::default());
    defmt::info!("starting");
    // Configure the USB stack
    let driver = usb::Driver::new(p.USB, Irqs);

    let mut config = embassy_usb::Config::new(0x045E, 0x0719);
    config.device_class = 0xFF;
    config.device_sub_class = 0xFF;
    config.device_protocol = 0xFF;
    config.device_release = 0x0100;
    config.manufacturer = Some("Timo Kröger");
    config.product = Some("wii-usb");
    config.serial_number = Some("FFFFFFFF");
    config.max_power = 260;
    config.max_packet_size_0 = 64;

    let mut device_descriptor = [0; 20];
    // Enough space to fit configuration descriptors for the controller interfaces
    // and the (optional) headset interfaces.
    let mut config_descriptor = [0; 324];
    let mut bos_descriptor = [0; 12];
    // Must be bigger than than n * 2 + 2 where n is the number of characters
    // in the longest string descriptor.
    let mut control_buf = [0; 64];

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut control_buf,
    );

    // The first 4 bytes should match the USB serial number descriptor.
    // Not required for the receiver to be detected by the windows driver.
    let mut serial_number_handler =
        xinput::SerialNumberHandler([0xFF, 0xFF, 0xFF, 0xFF, 0x0a, 0x89, 0xB7]);
    builder.handler(&mut serial_number_handler);

    let mut c0 = XInput::new_wireless(&mut builder, &CONTROLLER_STATE[0], false);

    let mut usb = builder.build();

    // USB
    let usb = pin!(async { usb.run().await });

    // Controllers
    let c0 = pin!(async move { c0.run().await });

    let ext_poll = pin!(async {
        let mut led = Output::new(p.PIN_20, Level::Low);

        let mut ticker = Ticker::every(Duration::from_millis(1));
        let sda = p.PIN_4;
        let scl = p.PIN_5;

        let i2c0 = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, i2c::Config::default());
        let mut classic = classic_async::ClassicAsync::new(i2c0);
        let _ = classic.init().await;
        let _ = classic.enable_hires().await;

        loop {
            ticker.next().await;
            led.set_low();
            let poll_data = classic.read().await;
            if poll_data.is_ok() {
                let poll_data = poll_data.expect("");
                let state = &CONTROLLER_STATE[0];
                state.send_xinput(wiiext_to_xinput(poll_data));
            }
            led.set_high();
        }
    });

    lilos::exec::run_tasks(&mut [usb, c0, ext_poll], lilos::exec::ALL_TASKS)
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    cortex_m::asm::udf();
}
