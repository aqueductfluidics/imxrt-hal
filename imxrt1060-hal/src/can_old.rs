//! CAN support
//!
//! The module provides an implementation of the `embedded_hal::can::nb::Can` trait.
//!
//! # Example
//!
//! ```no_run
//! use imxrt1060_hal;
//! use embedded_hal::blocking::spi::Transfer;
//!
//! let mut peripherals = imxrt1060_hal::Peripherals::take().unwrap();
//!
//! let (_, _, _, spi4_builder) = peripherals.spi.clock(
//!     &mut peripherals.ccm.handle,
//!     imxrt1060_hal::ccm::spi::ClockSelect::Pll2,
//!     imxrt1060_hal::ccm::spi::PrescalarSelect::LPSPI_PODF_5,
//! );
//!
//! let mut spi4 = spi4_builder.build(
//!     peripherals.iomuxc.b0.p02,
//!     peripherals.iomuxc.b0.p01,
//!     peripherals.iomuxc.b0.p03,
//! );
//!
//! spi4.enable_chip_select_0(peripherals.iomuxc.b0.p00);
//!
//! spi4.set_clock_speed(imxrt1060_hal::spi::ClockSpeed(1_000_000)).unwrap();
//!
//! let mut buffer: [u8; 3] = [1, 2, 3];
//! spi4.transfer(&mut buffer).unwrap();
//! ```

use crate::iomuxc::consts::{Unsigned, U1, U2};

use crate::ccm;
use crate::ral;
use core::marker::PhantomData;

use crate::can_old;

/// Unclocked CAN modules
///
/// The `Unclocked` struct represents the unconfigured CAN peripherals.
/// Once clocked, you'll have the ability to build CAN peripherals from the
/// compatible processor pins.
pub struct Unclocked {
    pub(crate) can1: ral::can::Instance,
    pub(crate) can2: ral::can::Instance,
}

impl Unclocked {
    /// Enable clocks to all CAN modules, returning a builder for the two CAN modules.
    pub fn clock(
        self,
        handle: &mut ccm::Handle,
        clock_select: ccm::can::ClockSelect,
        divider: ccm::can::PrescalarSelect,
    ) -> (Builder<U1>, Builder<U2>) {
        let (ccm, _) = handle.raw();
        // First, disable clocks
        // First, disable clocks
        ral::modify_reg!(
            ral::ccm,
            ccm,
            CCGR1,
            CG0: 0,
            CG1: 0,
            CG2: 0,
            CG3: 0
        );

        let clk_sel = match clock_select {
            ccm::can::ClockSelect::Pll2 => ral::ccm::CSCMR2::CAN_CLK_SEL::RW::CAN_CLK_SEL_1,
        };

        // Select clock, and commit prescalar
        ral::modify_reg!(
            ral::ccm,
            ccm,
            CSCMR2,
            CAN_CLK_PODF: 0,
            CAN_CLK_SEL: clk_sel
        );

        // Enable clocks
        ral::modify_reg!(
            ral::ccm,
            ccm,
            CCGR1,
            CG0: 0b11,
            CG1: 0b11,
            CG2: 0b11,
            CG3: 0b11
        );

        let source_clock = ccm::Frequency::from(clock_select) / ccm::Divider::from(divider);
        (
            Builder::new(source_clock, self.can1),
            Builder::new(source_clock, self.can2),
        )
    }
}

/// A SPI builder that can build a CAN peripheral
pub struct Builder<M> {
    _module: PhantomData<M>,
    reg: ral::can::Instance,
    /// Frequency of the LPSPI source clock. This
    /// accounts for the divider.
    source_clock: ccm::Frequency,
}

impl<M> Builder<M>
where
    M: Unsigned,
{
    fn new(source_clock: ccm::Frequency, reg: ral::can::Instance) -> Self {
        Builder {
            _module: PhantomData,
            reg,
            source_clock,
        }
    }

    /// Builds a CAN peripheral. The return
    /// is a configured CAN master running at 24MHz.
    pub fn build(self) -> CAN<M> {
        CAN::new(self.source_clock, self.reg)
    }
}

/// SPI Clock speed, in Hz
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(transparent)]
pub struct ClockSpeed(pub u32);

impl Default for ClockSpeed {
    fn default() -> Self {
        ClockSpeed(24_000_000)
    }
}

impl ClockSpeed {
    /// Sets the clock speed parameters
    ///
    /// # Safety
    ///
    /// The function touches SPI registers that should only be touched
    /// while the SPI master is disabled.
    unsafe fn set(self, source_clock: ccm::Frequency, reg: &ral::lpspi::Instance) {
        log::debug!(
            "CAN baud rate = {:?}, source clock = {:?}",
            self,
            source_clock
        );

        let mut div = source_clock.0 / self.0;

        if source_clock.0 / div > self.0 {
            div += 1;
        }

        // 0 <= div <= 255, and the true coefficient is really div + 2
        let div = div.saturating_sub(2).min(255).max(0);
        ral::write_reg!(
            ral::lpspi,
            reg,
            CCR,
            SCKDIV: div,
            // Both of these delays are arbitrary choices, and they should
            // probably be configurable by the end-user.
            DBT: div / 2,
            SCKPCS: 0x1F,
            PCSSCK: 0x1F
        );
    }
}

/// A CAN master
///
/// By default, the SPI master runs at 8Mhz, Use `set_clock_speed` to vary
/// the SPI bus speed.
pub struct CAN<M> {
    reg: ral::can::Instance,
    _module: PhantomData<M>,
    /// LPSPI effective input clock frequency
    source_clock: ccm::Frequency,
}

/// Indicates an error when computing the parameters that control
/// the clock speed.
#[derive(Debug)]
pub struct ClockSpeedError(());

/// Indicates an error when computing the parameters that control
/// the baud rate.
#[derive(Debug)]
pub struct BaudRateError(());

/// Indicates an error when computing the parameters that control
/// the pin low timeout
#[derive(Debug)]
pub struct PinLowTimeoutError(());

/// Indicates an error when computing the parameters that control
/// the bus idle timeout
#[derive(Debug)]
pub struct BusIdleTimeoutError(());

const RETRIES: usize = 100_000;

impl<M> CAN<M>
where
    M: Unsigned,
{
    const DMA_DESTINATION_REQUEST_SIGNAL: u32 = DMA_TX_REQUEST_LOOKUP[M::USIZE - 1];
    const DMA_SOURCE_REQUEST_SIGNAL: u32 = DMA_RX_REQUEST_LOOKUP[M::USIZE - 1];

    fn new(source_clock: ccm::Frequency, reg: ral::can::Instance) -> Self {
        let mut can = CAN {
            reg,
            _module: PhantomData,
            source_clock,
        };
        can.set_clock_speed(ClockSpeed::default()).unwrap();
        can
    }

    /// Set the baud rate for the CAN peripheral
    pub fn set_baud_rate(&mut self, baud_rate: u32) -> Result<(), BaudRateError> {
        Ok(())
    }

    #[inline(always)]
    fn wait<F>(&mut self, mut on: F) -> Result<(), Error>
    where
        F: FnMut(u32) -> bool,
    {
        for _ in 0..RETRIES {
            if on(self.check_errors()?) {
                return Ok(());
            }
        }
        Err(Error::WaitTimeout)
    }

    fn clear_status(&mut self) {
        
    }

    pub fn clear_fifo(&mut self) {
        ral::modify_reg!(ral::lpspi, self.reg, CR, RRF: RRF_1, RTF: RTF_1);
    }

    /// Check master status flags for erroneous conditions
    #[inline(always)]
    fn check_errors(&mut self) -> Result<u32, Error> {
        use ral::lpspi::SR::*;
        let status = ral::read_reg!(ral::lpspi, self.reg, SR);
        if status & TEF::mask != 0 {
            Err(Error::Transmit)
        } else if status & REF::mask != 0 {
            Err(Error::Receive)
        } else if status & DMF::mask != 0 {
            Err(Error::DataMismatch) // TODO: is this an error?
        } else {
            Ok(status)
        }
    }

    /// # Safety
    ///
    /// Interior mutability must be atomic
    #[inline(always)]
    unsafe fn set_frame_size<Word>(&self) {
        ral::modify_reg!(ral::lpspi, self.reg, TCR, FRAMESZ: ((core::mem::size_of::<Word>() * 8 - 1) as u32));
    }

    #[inline(always)]
    fn send<Word: Into<u32> + Copy>(&mut self, word: Word) -> nb::Result<(), Error> {
        use ral::lpspi::SR::*;

        let sr = self.check_errors()?;
        self.clear_status();
        // Safety: user provided mutable reference to SPI, so they are ensuring that
        // we can safely change this.
        unsafe { self.set_frame_size::<Word>() };

        if (sr & MBF::mask != 0) || (sr & TDF::mask == 0) {
            return Err(nb::Error::WouldBlock);
        }

        ral::write_reg!(ral::lpspi, self.reg, TDR, DATA: word.into());
        self.wait(|msr| msr & TDF::mask != 0)?;
        Ok(())
    }

    #[inline(always)]
    fn read(&mut self) -> nb::Result<u32, Error> {
        use ral::lpspi::SR::*;
        let sr = self.check_errors()?;
        if sr & MBF::mask != 0 {
            return Err(nb::Error::WouldBlock);
        }

        if ral::read_reg!(ral::lpspi, self.reg, RSR, RXEMPTY == RXEMPTY_0) {
            let word = ral::read_reg!(ral::lpspi, self.reg, RDR, DATA);
            Ok(word)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Perform common actions for enabling a DMA source
    ///
    /// # Safety
    ///
    /// Interior mutability must be atomic
    #[inline(always)]
    unsafe fn enable_dma_source<W>(&self) {
        self.set_frame_size::<W>();
        ral::modify_reg!(ral::lpspi, self.reg, FCR, RXWATER: 0); // No watermarks; affects DMA signaling
        ral::modify_reg!(ral::lpspi, self.reg, DER, RDDE: 1);
    }

    /// Perform common actions for disabling a DMA source
    ///
    /// # Safety
    ///
    /// Performs writes behind an immutable receiver. Interior mutability must be atomic.
    #[inline(always)]
    unsafe fn disable_dma_source(&self) {
        while ral::read_reg!(ral::lpspi, self.reg, DER, RDDE == 1) {
            ral::modify_reg!(ral::lpspi, self.reg, DER, RDDE: 0);
        }
    }

    /// # Safety
    ///
    /// Performs writes behind an immutable receiver. Interior mutability must be atomic.
    #[inline(always)]
    unsafe fn enable_dma_destination<W>(&self) {
        self.set_frame_size::<W>();
        ral::modify_reg!(ral::lpspi, self.reg, FCR, TXWATER: 0); // No watermarks; affects DMA signaling
        ral::modify_reg!(ral::lpspi, self.reg, DER, TDDE: 1);
    }

    /// # Safety
    ///
    /// Performs writes behind an immutable receiver. Interior mutability must be atomic.
    #[inline(always)]
    unsafe fn disable_dma_destination(&self) {
        while ral::read_reg!(ral::lpspi, self.reg, DER, TDDE == 1) {
            ral::modify_reg!(ral::lpspi, self.reg, DER, TDDE: 0);
        }
    }
}

/// An error that occured during a SPI operation
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// A generic transmit error
    Transmit,
    /// A generic receive error
    Receive,
    /// Data mismatch error
    DataMismatch,
    /// Busy-wait on an internal flag was too long
    WaitTimeout,
}

impl<M> embedded_hal::can::Can for CAN<M>
where
    M: Unsigned,
{
    type Frame = embedded_hal::can::Frame;

    type Error = embedded_hal::can::Error;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        let data: [u8; 8] = [0];
        Ok(Some(embedded_hal::can::Frame::new(0x00, &data)))
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        let data: [u8; 8] = [255];
        Ok(embedded_hal::can::Frame::new(0xFF, &data))
    }
}

//
// DMA peripheral support
//

use crate::dma;

/// SPI RX DMA Request signal
///
/// See table 4-3 of the iMXRT1060 Reference Manual (Rev 2)
const DMA_RX_REQUEST_LOOKUP: [u32; 4] = [13, 77, 15, 79];

/// SPI TX DMA Request signal
///
/// See table 4-3 of the iMXRT1060 Reference Manual (Rev 2)
const DMA_TX_REQUEST_LOOKUP: [u32; 4] = [14, 78, 16, 80];

unsafe impl<M> dma::peripheral::Source<u8> for CAN<M>
where
    M: Unsigned,
{
    fn source_signal(&self) -> u32 {
        Self::DMA_SOURCE_REQUEST_SIGNAL
    }
    fn source(&self) -> *const u8 {
        &self.reg.RDR as *const _ as *const u8
    }
    fn enable_source(&self) {
        cortex_m::interrupt::free(|_| unsafe {
            // Safety: atomic operation
            self.enable_dma_source::<u8>();
        });
    }
    fn disable_source(&self) {
        cortex_m::interrupt::free(|_| unsafe {
            // Safety: atomic operation
            self.disable_dma_source();
        });
    }
}

unsafe impl<M> dma::peripheral::Destination<u8> for CAN<M>
where
    M: Unsigned,
{
    fn destination_signal(&self) -> u32 {
        Self::DMA_DESTINATION_REQUEST_SIGNAL
    }
    fn destination(&self) -> *const u8 {
        &self.reg.TDR as *const _ as *const u8
    }
    fn enable_destination(&self) {
        cortex_m::interrupt::free(|_| unsafe {
            // Safety: atomic operation
            self.enable_dma_destination::<u8>();
        });
    }
    fn disable_destination(&self) {
        cortex_m::interrupt::free(|_| unsafe {
            // Safety: atomic operation
            self.disable_dma_destination();
        });
    }
}

unsafe impl<M> dma::peripheral::Source<u16> for CAN<M>
where
    M: Unsigned,
{
    fn source_signal(&self) -> u32 {
        Self::DMA_SOURCE_REQUEST_SIGNAL
    }
    fn source(&self) -> *const u16 {
        &self.reg.RDR as *const _ as *const u16
    }
    fn enable_source(&self) {
        cortex_m::interrupt::free(|_| unsafe {
            // Safety: atomic operation
            self.enable_dma_source::<u16>();
        });
    }
    fn disable_source(&self) {
        cortex_m::interrupt::free(|_| unsafe {
            // Safety: atomic operation
            self.disable_dma_source();
        });
    }
}

unsafe impl<M> dma::peripheral::Destination<u16> for CAN<M>
where
    M: Unsigned,
{
    fn destination_signal(&self) -> u32 {
        Self::DMA_DESTINATION_REQUEST_SIGNAL
    }
    fn destination(&self) -> *const u16 {
        &self.reg.TDR as *const _ as *const u16
    }
    fn enable_destination(&self) {
        cortex_m::interrupt::free(|_| unsafe {
            // Safety: atomic operation
            self.enable_dma_destination::<u16>();
        });
    }
    fn disable_destination(&self) {
        cortex_m::interrupt::free(|_| unsafe {
            // Safety: atomic operation
            self.disable_dma_destination();
        });
    }
}
