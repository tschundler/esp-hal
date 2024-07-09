//! This adapter allows for the use of an RMT output channel to easily interact
//! with RGB LEDs and use the convenience functions of the
//! [`smart-leds`](https://crates.io/crates/smart-leds) crate.
//!
//! This is a simple implementation where every LED is adressed in an
//! individual RMT operation. This is working perfectly fine in blocking mode,
//! but in case this is used in combination with interrupts that might disturb
//! the sequential sending, an alternative implementation (addressing the LEDs
//! in a sequence in a single RMT send operation) might be required!
//!
//! ## Example
//!
//! ```rust,ignore
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! let rmt = Rmt::new(peripherals.RMT, 80.MHz(), &clocks, None).unwrap();
//!
//! let rmt_buffer = smartLedBuffer!(1);
//! let mut led = SmartLedsAdapter::new(rmt.channel0, io.pins.gpio2, rmt_buffer, &clocks);
//! ```
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![deny(missing_docs)]
#![no_std]

use core::{fmt::Debug, iter::FusedIterator, slice::IterMut};

use esp_hal::{
    clock::Clocks,
    gpio::OutputPin,
    peripheral::Peripheral,
    rmt::{Error as RmtError, PulseCode, TxChannel, TxChannelConfig, TxChannelCreator},
};
use smart_leds_trait::{SmartLedsWrite, RGB8};

const SK68XX_CODE_PERIOD: u32 = 1200;
const SK68XX_T0H_NS: u32 = 320;
const SK68XX_T0L_NS: u32 = SK68XX_CODE_PERIOD - SK68XX_T0H_NS;
const SK68XX_T1H_NS: u32 = 640;
const SK68XX_T1L_NS: u32 = SK68XX_CODE_PERIOD - SK68XX_T1H_NS;
const SK68XX_TLATCH_NS: u32 = 20000;

/// All types of errors that can happen during the conversion and transmission
/// of LED commands
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LedAdapterError {
    /// Raised in the event that the provided data container is not large enough
    BufferSizeExceeded,
    /// Raised if something goes wrong in the transmission,
    TransmissionError(RmtError),
}

/// Macro to allocate a buffer sized for a specific number of LEDs to be
/// addressed.
///
/// Attempting to use more LEDs that the buffer is configured for will result in
/// an `LedAdapterError:BufferSizeExceeded` error.
#[macro_export]
macro_rules! smartLedBuffer {
    ( $buffer_size: literal ) => {
        // The size we're assigning here is calculated as following
        //  (
        //   Nr. of LEDs
        //   * channels (r,g,b -> 3)
        //   * pulses per channel 8)
        //  ) + 1 additional pulse for the end delimiter
        [0u32; $buffer_size * 24 + 1]
    };
}

struct ByteGenerator<T>
where
    T: Iterator<Item = RGB8>,
{
    source: T,
    val: RGB8,
    bit: u8,
}

impl<T> Iterator for ByteGenerator<T>
where
    T: Iterator<Item = RGB8>,
{
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        if self.bit == 0 {
            self.val = self.source.next()?;
            self.bit = 3;
        }
        self.bit -= 1;
        Some(match self.bit {
            0 => self.val.b,
            1 => self.val.r,
            _ => self.val.g,
        })
    }
}

impl<T> ByteGenerator<T>
where
    T: Iterator<Item = RGB8>,
{
    fn new(source: T) -> Self {
        Self {
            source,
            val: RGB8 { r: 0, g: 0, b: 0 },
            bit: 0,
        }
    }
}

enum PulseState {
    Running,
    SendStop,
    Done,
}

struct PulseGenerator<T>
where
    T: Iterator<Item = u8>,
{
    source: T,
    pulses: (u32, u32),
    bit: u8,
    val: u8,
    state: PulseState,
}

impl<T> PulseGenerator<T>
where
    T: Iterator<Item = u8>,
{
    fn new(source: T, pulses: (u32, u32)) -> Self {
        Self {
            source,
            pulses,
            bit: 0,
            val: 0,
            state: PulseState::Running,
        }
    }
}

impl<T> Iterator for PulseGenerator<T>
where
    T: Iterator<Item = u8>,
{
    type Item = u32;

    #[inline(never)]
    fn next(&mut self) -> Option<Self::Item> {
        if self.bit == 0 {
            match self.state {
                PulseState::Running => {
                    let Some(v) = self.source.next() else {
                        self.state = PulseState::SendStop;
                        // should send latch delay
                        return Some(0);
                    };
                    self.val = v;
                    self.bit = 8;
                }
                PulseState::SendStop => {
                    self.state = PulseState::Done;
                    return Some(0);
                }
                PulseState::Done => return None,
            }
        }
        let result = if self.val & 128 != 0 {
            self.pulses.1
        } else {
            self.pulses.0
        };
        self.bit -= 1;
        self.val <<= 1;

        Some(result)
    }
}

impl<T> FusedIterator for PulseGenerator<T> where T: Iterator<Item = u8> {}

/// Adapter taking an RMT channel and a specific pin and providing RGB LED
/// interaction functionality using the `smart-leds` crate
pub struct SmartLedsAdapter<TX>
where
    TX: TxChannel,
{
    channel: Option<TX>,
    pulses: (u32, u32),
}

impl<'d, TX> SmartLedsAdapter<TX>
where
    TX: TxChannel,
{
    /// Create a new adapter object that drives the pin using the RMT channel.
    pub fn new<C, O>(
        channel: C,
        pin: impl Peripheral<P = O> + 'd,
        clocks: &Clocks,
    ) -> SmartLedsAdapter<TX>
    where
        O: OutputPin + 'd,
        C: TxChannelCreator<'d, TX, O>,
    {
        let config = TxChannelConfig {
            clk_divider: 1,
            idle_output_level: false,
            carrier_modulation: false,
            idle_output: true,

            ..TxChannelConfig::default()
        };

        let channel = channel.configure(pin, config).unwrap();

        // Assume the RMT peripheral is set up to use the APB clock
        let src_clock = clocks.apb_clock.to_MHz();

        Self {
            channel: Some(channel),
            pulses: (
                u32::from(PulseCode {
                    level1: true,
                    length1: ((SK68XX_T0H_NS * src_clock) / 1000) as u16,
                    level2: false,
                    length2: ((SK68XX_T0L_NS * src_clock) / 1000) as u16,
                }),
                u32::from(PulseCode {
                    level1: true,
                    length1: ((SK68XX_T1H_NS * src_clock) / 1000) as u16,
                    level2: false,
                    length2: ((SK68XX_T1L_NS * src_clock) / 1000) as u16,
                }),
            ),
        }
    }
}

impl<TX> SmartLedsWrite for SmartLedsAdapter<TX>
where
    TX: TxChannel,
{
    type Error = LedAdapterError;
    type Color = RGB8;

    /// Convert all RGB8 items of the iterator to the RMT format and
    /// add them to internal buffer, then start a singular RMT operation
    /// based on that buffer.
    fn write<T, I>(&mut self, iterator: T) -> Result<(), Self::Error>
    where
        T: IntoIterator<Item = I>,
        I: Into<Self::Color>,
    {
        let channel = self.channel.take().unwrap();
        let mut itr = PulseGenerator::new(
            ByteGenerator::new(iterator.into_iter().map(|v| v.into())),
            self.pulses,
        );
        match channel.transmit(&mut itr).wait() {
            Ok(chan) => {
                self.channel = Some(chan);
                Ok(())
            }
            Err((e, chan)) => {
                self.channel = Some(chan);
                Err(LedAdapterError::TransmissionError(e))
            }
        }
    }
}
