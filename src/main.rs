//! Draw an RGB565 BMP image onto the display.
//!
//! This example is for the Raspberry Pi Pico board.
//!
//! Converted using `convert rust.png -type truecolor -define bmp:subtype=RGB565 rust.bmp`

#![no_std]
#![no_main]

use defmt_rtt as _; // global logger

// TODO(5) adjust HAL import
// use some_hal as _; // memory layout 

use panic_probe as _;

mod qmi8658c;

//use core::fmt::Write as fmt_write;
use heapless::String;

use eg_seven_segment::SevenSegmentStyleBuilder;

use rp_pico as bsp;

use bsp::entry;
use fugit::RateExtU32;

use display_interface_spi::SPIInterface;
use embedded_graphics::prelude::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    image::Image, 
    pixelcolor::Rgb565,
    primitives::{Circle, PrimitiveStyleBuilder, Rectangle, Triangle},
    text::{Alignment, Text},
};
use tinybmp::Bmp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio, pac, pwm,
    sio::Sio,
    spi,
    I2C,
    usb,
    watchdog::Watchdog,
};

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

use crate::qmi8658c::QMI8658C;

// I2C HAL traits & Types.
use embedded_hal::blocking::i2c::{Operation, Read, Transactional, Write};

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio10.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<gpio::FunctionSpi>();
    let spi_cs = pins.gpio9.into_push_pull_output();

    // Create an SPI driver instance for the SPI1 device
    let spi = spi::Spi::<_, _, 8>::new(pac.SPI1);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        62_500_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let dc_pin = pins.gpio8.into_push_pull_output();
    let rst_pin = pins.gpio12.into_push_pull_output();

    let spi_interface = SPIInterface::new(spi, dc_pin, spi_cs);

    // initialize PWM for backlight
    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM6
    let mut pwm = pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM2 to GPIO 13
    let mut channel = pwm.channel_b;
    channel.output_to(pins.led);

    // Create display driver
    let mut display = gc9a01a::GC9A01A::new(spi_interface, rst_pin, channel);
    // Bring out of reset
    display.reset(&mut delay).unwrap();
    // Turn on backlight
    display.set_backlight(55000);
    // Initialize registers
    display.initialize(&mut delay).unwrap();
    // Clear the screen
    display.clear(Rgb565::BLACK).unwrap();

    let sda_pin = pins.gpio6.into_mode::<gpio::FunctionI2C>();
    let scl_pin = pins.gpio7.into_mode::<gpio::FunctionI2C>();

    let i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut qmi8658c = QMI8658C::new(i2c);
    qmi8658c.init().unwrap();

    let style = PrimitiveStyleBuilder::new()
        .stroke_width(2)
        .stroke_color(Rgb565::CSS_RED)
        .build();

    // screen outline for the round 1.28 inch Waveshare display
    Circle::new(Point::new(1, 1), 238)
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    let Ok(bmp) = Bmp::from_slice(include_bytes!("./rust.bmp")) else {
        display.clear(Rgb565::RED).unwrap();
        exit()
    };

    // The image is an RGB565 encoded BMP, so specifying the type as `Image<Bmp<Rgb565>>`
    // will read the pixels correctly
    let im: Image<Bmp<Rgb565>> = Image::new(&bmp, Point::new(56, 56));

    im.draw(&mut display).unwrap();
    
    // Create a Text object and draw it to the display
    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    let text_bounds = Rectangle::new(Point::zero(), display.bounding_box().size);
    let text = "Hello, World!";
    let text_position = Point::new(50, 200);
    Text::new(text, text_position, text_style).draw(&mut display).unwrap();
    
    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Kouvosto Telecom")
        .product("RP-128")
        .serial_number("001")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    
    // Define a new style.
    let seg_style = SevenSegmentStyleBuilder::new()
        .digit_size(Size::new(30, 50)) // digits are 10x20 pixels
        .digit_spacing(5)              // 5px spacing between digits
        .segment_width(5)              // 5px wide segments
        .segment_color(Rgb565::GREEN)  // active segments are green
        .build();


    Text::new("12:34", Point::new(30, 120), seg_style).draw(&mut display).unwrap();

    let mut i: i32 = 0;
    
    delay.delay_ms(1000);

    loop {

        defmt::println!("Hello World!");

        display.clear(Rgb565::BLACK).unwrap();

        let accel_gyro_data = qmi8658c.read_accel_gyro().unwrap_or_default();
        //println!("Accel: ({}, {}, {})", accel_gyro_data.accel_x, accel_gyro_data.accel_y, accel_gyro_data.accel_z);
        //println!("Gyro: ({}, {}, {})", accel_gyro_data.gyro_x, accel_gyro_data.gyro_y, accel_gyro_data.gyro_z);

        //let mut data = String::<32>::new(); // 32 byte string buffer
        
        // `write` for `heapless::String` returns an error if the buffer is full,
        // but because the buffer here is 32 bytes large, the u32 will fit with a 
        // lot of space left. You can shorten the buffer if needed to save space.
        //let _ = write!(data, "step:{i}");

        // Use the style to draw text to a embedded-graphics `DrawTarget`.
        
        let _ = serial.write(b"Hello World");
        //Text::new(&data, text_position, text_style).draw(&mut display).unwrap();

        Circle::new(Point::new(i, i), 100)
            .into_styled(style)
            .draw(&mut display)
            .unwrap();

        if i >= 120{
            i -= 1;
        }
        else {
            i += 1;
        }
    
        delay.delay_ms(10);
    }
}

pub fn exit() -> ! {
    loop {}
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
