//! GPIOs
//!
//! This GPIO driver supports the `embedded_hal`'s `v2` digital traits.
//!
//! # Fast and Normal GPIOs
//!
//! High speed, or "fast," GPIOs are GPIOs that run on the AHB clock. Normal GPIOs
//! run on the IPG clock.
//!
//! # Example
//!
//! ```no_run
//! use imxrt1060_hal::{self, gpio::GPIO};
//!
//! let mut peripherals = imxrt1060_hal::Peripherals::take().unwrap();
//! let input = GPIO::new(peripherals.iomuxc.ad_b0.p11);
//!
//! assert!(!input.is_set());
//! let mut output = input.output();
//!
//! output.set();
//! assert!(output.is_set());
//!
//! output.toggle();
//! assert!(!output.is_set());
//!
//! assert!(output.set_fast(true));
//! output.toggle();
//! assert!(output.is_set());
//! ```
//!
//! # Interrupts
//!
//! GPIO inputs can generate interrupts on edge or level triggers. See
//! [`InterruptConfiguration`](crate::gpio::InterruptConfiguration) to
//! understand the available configurations.
//!
//! Before selecting a GPIO interrupt vector, you need to know which GPIO
//! port and pin is associated with your pad. There's two ways to correlate
//! a pad to a GPIO:
//!
//! - Check the reference manual. Chapter 10 of the i.MX RT 1060 reference manual has a table that associates
//!   pads to peripheral functions.
//! - Use the `imxrt-iomuxc` crate's documentation. Browse to [the `gpio::Pin` documentation](https://docs.rs/imxrt-iomuxc/0.1.3/imxrt_iomuxc/gpio/trait.Pin.html),
//!   and scroll down to see the implementors. Example: expand the implementation for [B0_06](https://docs.rs/imxrt-iomuxc/0.1.3/imxrt_iomuxc/gpio/trait.Pin.html#impl-Pin-38),
//!   and it reveals the GPIO port and pin (`GPIO2_6`).
//!
//! Depending on your specific i.MX RT variant, there may be various GPIO interrupt
//! vectors. To implement the most portable code, consider using the
//!
//! - `GPIO[X]_Combined_0_15`
//! - `GPIO[X]_Combined_16_31`
//!
//! interrupts, replacing `[X]` with your GPIO port number. These two interrupts handle
//! GPIO pins 0 through 15, and GPIO pins 16 through 31, respectively.
//!
//! To define the interrupt, use the APIs from [the `cortex-m-rt` crate](https://docs.rs/cortex-m-rt/0.7.0/cortex_m_rt/).
//!
//! The snippet below demonstrates a function that defines an interrupt. The interrupt
//! invokes a user's callback when input pin detects a rising edge.
//!
//! ```no_run
//! use imxrt1060_hal as hal;
//! use hal::{
//!     gpio::{GPIO, Input, InterruptConfiguration},
//!     iomuxc::imxrt1060::b0::B0_10,
//! };
//! use cortex_m::interrupt::Mutex;
//! use core::cell::RefCell;
//! use hal::ral::interrupt;
//!
//! /// B0_10 => GPIO2_6, so we need to register
//! /// the GPIO2_Combined_0_15 interrupt.
//! type InputPin = GPIO<B0_10, Input>;
//! /// The response when the interrupt fires.
//! /// The callback runs within a critical section.
//! type InterruptCallback = fn(&cortex_m::interrupt::CriticalSection);
//!
//! fn setup_gpio_interrupt(mut pin: InputPin, callback: InterruptCallback) {
//!     struct SharedState { pin: InputPin, callback: InterruptCallback }
//!     static SHARED_STATE: Mutex<RefCell<Option<SharedState>>> = Mutex::new(RefCell::new(None));
//!
//!     #[cortex_m_rt::interrupt]
//!     fn GPIO2_Combined_0_15() {
//!         cortex_m::interrupt::free(|cs| {
//!             SHARED_STATE.borrow(cs).borrow_mut().as_mut().map(|state| {
//!                 if state.pin.is_interrupt_status() {
//!                     state.pin.clear_interrupt_status();
//!                     (state.callback)(cs);
//!                 }
//!             });
//!         });
//!     }
//!
//!     cortex_m::interrupt::free(|cs| {
//!         pin.set_interrupt_configuration(InterruptConfiguration::RisingEdge);
//!         pin.set_interrupt_enable(true);
//!
//!         *SHARED_STATE.borrow(cs).borrow_mut() = Some(SharedState{ pin, callback });
//!         // Safety: shared state is set within a critical section.
//!         unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::GPIO2_Combined_0_15) };
//!     });
//! }
//! ```

use crate::iomuxc::{consts::Unsigned, gpio::Pin};
use crate::ral::{
    self,
    gpio::{self, RegisterBlock},
};
use core::marker::PhantomData;

/// Denotes that a pin is configured as an input
pub enum Input {}
/// Denotes that a pin is configured as an output
pub enum Output {}

pub struct GPIO<P, D> {
    pin: P,
    dir: PhantomData<D>,
}

impl<P, D> GPIO<P, D>
where
    P: Pin,
{
    fn register_block(&self) -> *const RegisterBlock {
        const REGISTER_BLOCKS: [*const RegisterBlock; 9] = [
            gpio::GPIO1,
            gpio::GPIO2,
            gpio::GPIO3,
            gpio::GPIO4,
            gpio::GPIO5,
            gpio::GPIO6,
            gpio::GPIO7,
            gpio::GPIO8,
            gpio::GPIO9,
        ];
        REGISTER_BLOCKS[self.module().wrapping_sub(1)]
    }

    /// Returns the bitmask for this GPIO
    #[inline(always)]
    fn mask(&self) -> u32 {
        1u32 << <P as Pin>::Offset::USIZE
    }

    /// Returns the ICR field offset for this GPIO
    ///
    /// ICR is "Interrupt Configuration Register"
    fn icr_offset(&self) -> usize {
        (<P as Pin>::Offset::USIZE % 16) * 2
    }

    /// Returns the ICR mask for this GPIO
    fn icr_mask(&self) -> u32 {
        0b11 << self.icr_offset()
    }

    /// The return is a non-zero number, since the GPIO identifiers
    /// start with '1.'
    #[inline(always)]
    fn module(&self) -> usize {
        let fast_offset = if self.is_fast() { 5 } else { 0 };
        <P as Pin>::Module::USIZE + fast_offset
    }

    fn gpr(&self) -> Option<*mut u32> {
        // GPR register for GPIO1
        const GPR26: *mut u32 = 0x400A_C068 as *mut u32;
        if <P as Pin>::Module::USIZE < 5 {
            // Safety: GPR registers in range for GPIO1, 2, 3, and 4
            Some(unsafe { GPR26.add(<P as Pin>::Module::USIZE.wrapping_sub(1)) })
        } else {
            None
        }
    }

    /// Returns `true` if the GPIO is configured for fast mode
    ///
    /// If the GPIO cannot support fast mode, `is_fast()` always returns `false`
    pub fn is_fast(&self) -> bool {
        // Safety: MMIO valid per gpr() function.
        // Read is atomic
        self.gpr()
            .map(|gpr| unsafe { core::ptr::read_volatile(gpr) & self.mask() != 0 })
            .unwrap_or(false)
    }

    /// Configures the GPIO to fast mode. `true` indicates "fast," and `false` indicates "normal."
    ///
    /// Returns `false` if this pin does not support fast mode. Otherwise, returns `true`, indicating
    /// that the setting was respected.
    ///
    /// If you transition an output pin into fast mode, the GPIO output state may *not* be maintained.
    /// That is, if your GPIO pin was high, a transition into fast mode may set the pin low. Consider
    /// setting fast mode before driving the GPIO output to avoid inconsistencies.
    pub fn set_fast(&mut self, fast: bool) -> bool {
        self.gpr()
            .map(|gpr| {
                cortex_m::interrupt::free(|_| unsafe {
                    let v = core::ptr::read_volatile(gpr);

                    let from = self.register_block();

                    if fast {
                        core::ptr::write_volatile(gpr, v | self.mask());
                    } else {
                        core::ptr::write_volatile(gpr, v & !self.mask());
                    }

                    let to = self.register_block();
                    self.copy_settings(from, to);

                    true
                })
            })
            .unwrap_or(false)
    }

    /// Copies the settings for one GPIO register block, `from`, to another register block, `to`.
    ///
    /// This method runs when changing from a normal to a fast GPIO, or a fast to a normal GPIO.
    /// The goal is to make the switch seamless for the end user. You must only copy the settings
    /// for the current GPIO pin.
    ///
    /// # Safety
    ///
    /// This method must run in a critical section, since it performs reads and writes across
    /// multiple register blocks which are shared across multiple pins.
    unsafe fn copy_settings(&self, from: *const RegisterBlock, to: *const RegisterBlock) {
        macro_rules! copy_bits {
            ($REG:ident, $mask:expr) => {{
                let source_register = ral::read_reg!(ral::gpio, from, $REG);
                let target_bits = $mask & source_register;

                ral::modify_reg!(ral::gpio, to, $REG, |mut destination| {
                    // Set the bit low...
                    destination &= !$mask;
                    // OR in the previous setting...
                    destination | target_bits
                });
            }};
        }

        // The input / output direction. When switching across fast / normal, keep the
        // same direction.
        copy_bits!(GDIR, self.mask());

        // Interrupt configuration is preserved when switching fast / normal.
        if <P as Pin>::Offset::USIZE < 16 {
            copy_bits!(ICR1, self.icr_mask());
        } else {
            copy_bits!(ICR2, self.icr_mask());
        }
        copy_bits!(EDGE_SEL, self.mask());
        copy_bits!(IMR, self.mask());
    }

    /// Configure the GPIO as an output
    fn set_output(&self, _: &cortex_m::interrupt::CriticalSection) {
        // Safety: critical section, enforced by API, ensures consistency
        unsafe {
            // Turn off interrupts for pin.
            ral::modify_reg!(ral::gpio, self.register_block(), IMR, |imr| imr
                & !self.mask());

            // Change direction
            ral::modify_reg!(ral::gpio, self.register_block(), GDIR, |gdir| gdir
                | self.mask());
        }
    }

    /// Configure the GPIO as an input
    fn set_input(&self, _: &cortex_m::interrupt::CriticalSection) {
        // Safety: critical section, enforced by API, ensures consistency
        unsafe {
            ral::modify_reg!(ral::gpio, self.register_block(), GDIR, |gdir| gdir
                & !self.mask());
        }
    }
}

impl<P> GPIO<P, Input>
where
    P: Pin,
{
    /// Create a GPIO from a pad that supports a GPIO configuration
    ///
    /// All pads may be used as a GPIO, so this should always work.
    pub fn new(mut pin: P) -> Self {
        crate::iomuxc::gpio::prepare(&mut pin);
        Self {
            pin,
            dir: PhantomData,
        }
    }

    /// Set the GPIO as an output.
    ///
    /// Any interrupt configuration will be cleared and needs redoing if the pin is transitioned
    /// back to an input.
    pub fn output(self) -> GPIO<P, Output> {
        cortex_m::interrupt::free(|cs| self.set_output(cs));
        GPIO {
            pin: self.pin,
            dir: PhantomData,
        }
    }

    /// Returns `true` if this input pin is high
    pub fn is_set(&self) -> bool {
        // Safety: read is atomic
        unsafe { ral::read_reg!(ral::gpio, self.register_block(), PSR) & self.mask() != 0 }
    }
}

/// GPIO input interrupt configurations.
///
/// These configurations do not take effect until
/// GPIO input interrupts are enabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum InterruptConfiguration {
    LowLevel = 0,
    HighLevel = 1,
    RisingEdge = 2,
    FallingEdge = 3,
    EitherEdge = 4,
}

impl<P> GPIO<P, Input>
where
    P: Pin,
{
    /// Enable (`true`) or disable (`false`) interrupts for this GPIO input.
    pub fn set_interrupt_enable(&mut self, enable: bool) {
        cortex_m::interrupt::free(|_| unsafe {
            ral::modify_reg!(ral::gpio, self.register_block(), IMR, |imr| if enable {
                imr | self.mask()
            } else {
                imr & !self.mask()
            })
        });
    }

    /// Indicates if interrupts are (`true`) or are not (`false`) enabled for this GPIO input.
    pub fn is_interrupt_enabled(&self) -> bool {
        unsafe { ral::read_reg!(ral::gpio, self.register_block(), IMR) & self.mask() != 0u32 }
    }

    /// Set the interrupt configuration for this GPIO input.
    pub fn set_interrupt_configuration(&mut self, interrupt_configuration: InterruptConfiguration) {
        // Safety: These modify_reg! must be completed as one unit, or we get an inconsistent state.
        cortex_m::interrupt::free(|_| unsafe {
            if InterruptConfiguration::EitherEdge == interrupt_configuration {
                ral::modify_reg!(ral::gpio, self.register_block(), EDGE_SEL, |edge_sel| {
                    edge_sel | self.mask()
                });
            } else {
                ral::modify_reg!(ral::gpio, self.register_block(), EDGE_SEL, |edge_sel| {
                    edge_sel & !self.mask()
                });
                let icr = interrupt_configuration as u32;
                let icr_modify = |reg| reg & !self.icr_mask() | (icr << self.icr_offset());
                if <P as Pin>::Offset::USIZE < 16 {
                    ral::modify_reg!(ral::gpio, self.register_block(), ICR1, icr_modify);
                } else {
                    ral::modify_reg!(ral::gpio, self.register_block(), ICR2, icr_modify);
                }
            }
        });
    }

    /// Indicates whether this GPIO input triggered an interrupt.
    pub fn is_interrupt_status(&self) -> bool {
        unsafe { ral::read_reg!(ral::gpio, self.register_block(), ISR) & self.mask() != 0u32 }
    }

    /// Clear the interrupt status flag.
    pub fn clear_interrupt_status(&mut self) {
        unsafe { ral::write_reg!(ral::gpio, self.register_block(), ISR, self.mask()) }
    }
}

impl<P> GPIO<P, Output>
where
    P: Pin,
{
    /// Transition the pin back to an input
    pub fn input(self) -> GPIO<P, Input> {
        cortex_m::interrupt::free(|cs| self.set_input(cs));
        GPIO {
            pin: self.pin,
            dir: PhantomData,
        }
    }

    /// Set the GPIO high
    pub fn set(&mut self) {
        // Safety: atomic write
        unsafe { ral::write_reg!(ral::gpio, self.register_block(), DR_SET, self.mask()) };
    }

    /// Set the GPIO low
    pub fn clear(&mut self) {
        // Safety: atomic write
        unsafe { ral::write_reg!(ral::gpio, self.register_block(), DR_CLEAR, self.mask()) };
    }

    /// Returns `true` if the pin is high
    pub fn is_set(&self) -> bool {
        // Safety: atomic read
        unsafe { ral::read_reg!(ral::gpio, self.register_block(), DR) & self.mask() != 0u32 }
    }

    /// Alternate the state of the pin
    pub fn toggle(&mut self) {
        // Safety: atomic write
        unsafe { ral::write_reg!(ral::gpio, self.register_block(), DR_TOGGLE, self.mask()) }
    }
}

use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, ToggleableOutputPin};

impl<P> OutputPin for GPIO<P, Output>
where
    P: Pin,
{
    type Error = core::convert::Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set();
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.clear();
        Ok(())
    }
}

impl<P> StatefulOutputPin for GPIO<P, Output>
where
    P: Pin,
{
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set())
    }
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        self.is_set_high().map(|res| !res)
    }
}

impl<P> ToggleableOutputPin for GPIO<P, Output>
where
    P: Pin,
{
    type Error = core::convert::Infallible;
    fn toggle(&mut self) -> Result<(), Self::Error> {
        GPIO::<P, Output>::toggle(self);
        Ok(())
    }
}

impl<P> InputPin for GPIO<P, Input>
where
    P: Pin,
{
    type Error = core::convert::Infallible;
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set())
    }
    fn is_low(&self) -> Result<bool, Self::Error> {
        self.is_high().map(|res| !res)
    }
}
