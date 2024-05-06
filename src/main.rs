#![deny(unsafe_code)]
#![no_main]
#![no_std]

use embedded_hal::pwm::SetDutyCycle;
// Halt on panic
use panic_halt as _;

// use core::fmt::Write; // for pretty formatting of the output
use cortex_m_rt::entry;
use mcp9808::{
    address::SlaveAddress, reg_conf::*, reg_res::*, reg_temp_generic::ReadableTempRegister, MCP9808,
};

use pid::Pid;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{
    i2c::Mode,
    pac,
    prelude::*,
    timer::{Channel1, Channel2, Polarity},
};

use nb::block;

const TEMP_SETTING_1: f32 = 45.0;
const TEMP_SETTING_2: f32 = 45.0;
const MAX_TEMP_1: f32 = 60.0;
const MAX_TEMP_2: f32 = 60.0;

#[allow(clippy::empty_loop)]
#[entry]
fn main() -> ! {
    rtt_init_print!();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let rcc = dp.RCC.constrain();
    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();
    // Acquire the GPIOA peripheral
    let gpioa = dp.GPIOA.split();
    // Acquire the GPIOB peripheral
    let gpiob = dp.GPIOB.split();

    // define RX/TX pins
    let tx_pin = gpioa.pa2;
    // configure serial
    // let mut tx = Serial::tx(dp.USART1, tx_pin, 115200.bps(), &clocks).unwrap();
    // or
    let mut tx = dp.USART2.tx(tx_pin, 115200.bps(), &clocks).unwrap();

    let scl = gpiob.pb6.into_alternate_open_drain();
    let sda = gpiob.pb7.into_alternate_open_drain();

    // let m_i2c1 = dp.I2C1.i2c(
    //     (scl, sda),
    //     Mode::Standard {
    //         frequency: 400_000_u32.Hz(),
    //     },
    //     &clocks,
    // );

    let mut i2c = stm32f4xx_hal::i2c::I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Standard {
            frequency: 400_000_u32.Hz(),
        },
        &clocks,
    );

    /* pwm generation ch1 and ch2 with tim2 */
    let channels = (
        Channel1::new(gpioa.pa0.internal_pull_up(true)),
        Channel2::new(gpioa.pa1.internal_pull_up(true)),
    );
    let pwm = dp.TIM2.pwm_hz(channels, 1.kHz(), &clocks).split();
    let (mut ch1, mut ch2) = pwm;
    ch1.set_duty_cycle_percent(0).unwrap();
    ch1.set_polarity(Polarity::ActiveLow);
    ch1.enable();
    ch2.set_duty_cycle_percent(0).unwrap();
    ch2.set_polarity(Polarity::ActiveLow);
    ch2.enable();

    /* pid ctrl */
    let mut pid_1 = Pid::new(TEMP_SETTING_1, 100.0);
    pid_1.p(11.5, 100.0);
    pid_1.i(0.13, 50.0);
    pid_1.d(0.1, 10.0);

    let mut pid_2 = Pid::new(TEMP_SETTING_2, 100.0);
    pid_2.p(11.5, 100.0);
    pid_2.i(0.13, 50.0);
    pid_2.d(0.1, 10.0);

    let mut delay = dp.TIM1.delay_ms(&clocks);
    loop {
        ////////////////////////////////////////////////////////////////////////////////////
        // begin 1
        let mut mcp9808 = MCP9808::new(
            i2c
        );

        mcp9808.set_address(SlaveAddress::Alternative { a2: true, a1: true, a0: true });

        // to read & write register
        let mut conf = mcp9808.read_configuration().unwrap();
        conf.set_shutdown_mode(ShutdownMode::Continuous);
        mcp9808.write_register(conf).unwrap();
        // read temperature register
        let temp_1_raw = mcp9808.read_temperature().unwrap().get_raw_value();
        let [high_1, low_1] = temp_1_raw.to_be_bytes();
        let temp_1 = mcp9808
            .read_temperature()
            .unwrap()
            .get_celsius(ResolutionVal::Deg_0_0625C);
        rprintln!("->temp_1: {:.4}", temp_1);

        i2c = mcp9808.free();

        // writeln!(tx, "temp: {}", temp_1).unwrap();
        // end 1

        // begin 2
        mcp9808 = MCP9808::new(
            i2c
        );

        mcp9808.set_address(SlaveAddress::Alternative { a2: false, a1: false, a0: false });

        // to read & write register
        conf = mcp9808.read_configuration().unwrap();
        conf.set_shutdown_mode(ShutdownMode::Continuous);
        mcp9808.write_register(conf).unwrap();
        // read temperature register
        let temp_2_raw = mcp9808.read_temperature().unwrap().get_raw_value();
        let [high_2, low_2] = temp_2_raw.to_be_bytes();
        let temp_2 = mcp9808
            .read_temperature()
            .unwrap()
            .get_celsius(ResolutionVal::Deg_0_0625C);
        rprintln!("->temp_2: {:.4}", temp_2);

        i2c = mcp9808.free();

        // writeln!(tx, "temp_2: {}", temp_2).unwrap();
        // end 2

        block!(tx.write(0xDE as u8)).unwrap();
        block!(tx.write(high_1)).unwrap();
        block!(tx.write(low_1)).unwrap();
        block!(tx.write(high_2)).unwrap();
        block!(tx.write(low_2)).unwrap();
        block!(tx.write(high_1 ^ low_1 ^ high_2 ^ low_2)).unwrap();
        ////////////////////////////////////////////////////////////////////////////////////

        // F
        let pid_f = pid_1.next_control_output(temp_1);
        if pid_f.output > 0.0 && pid_f.output <= 100.0 && temp_1 < MAX_TEMP_1 {
            ch1.set_duty_cycle_percent(pid_f.output as u8).unwrap();
            rprintln!("[pid_f]: {:.4}", pid_f.output);
        } else {
            ch1.set_duty_cycle_percent(0).unwrap();
            rprintln!("[pid_f]: {:.4}", pid_f.output);
        }

        // X
        let pid_x = pid_2.next_control_output(temp_2);
        if pid_x.output > 0.0 && pid_x.output <= 100.0 && temp_2 < MAX_TEMP_2 {
            ch2.set_duty_cycle_percent(pid_x.output as u8).unwrap();
            rprintln!("[pid_x]: {:.4}", pid_x.output);
        } else {
            ch2.set_duty_cycle_percent(0).unwrap();
            rprintln!("[pid_x]: {:.4}", pid_x.output);
        }

        delay.delay(250.millis());
    }
}
