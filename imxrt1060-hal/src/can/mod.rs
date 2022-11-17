mod embedded_hal;
pub mod filter;
mod frame;
mod id;
mod interrupt;

pub use id::{ExtendedId, Id, StandardId};

pub use frame::{Data, Frame, FramePriority};

use crate::iomuxc::consts::{Unsigned, U1, U2};
use crate::ccm;
use crate::ral;

use core::cmp::{Ord, Ordering};
use core::convert::{Infallible, TryInto};
use core::marker::PhantomData;
use core::mem;
use core::ptr::NonNull;

/// Error that indicates that an incoming message has been lost due to buffer overrun.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub struct OverrunError {
    _priv: (),
}

/// Identifier of a CAN message.
///
/// Can be either a standard identifier (11bit, Range: 0..0x3FF) or a
/// extendended identifier (29bit , Range: 0..0x1FFFFFFF).
///
/// The `Ord` trait can be used to determine the frame’s priority this ID
/// belongs to.
/// Lower identifier values have a higher priority. Additionally standard frames
/// have a higher priority than extended frames and data frames have a higher
/// priority than remote frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub struct IdReg(u32);

impl IdReg {
    const STANDARD_SHIFT: u32 = 21;

    const EXTENDED_SHIFT: u32 = 3;

    const IDE_MASK: u32 = 0x0000_0004;

    const RTR_MASK: u32 = 0x0000_0002;

    /// Creates a new standard identifier (11bit, Range: 0..0x7FF)
    ///
    /// Panics for IDs outside the allowed range.
    fn new_standard(id: StandardId) -> Self {
        Self(u32::from(id.as_raw()) << Self::STANDARD_SHIFT)
    }

    /// Creates a new extendended identifier (29bit , Range: 0..0x1FFFFFFF).
    ///
    /// Panics for IDs outside the allowed range.
    fn new_extended(id: ExtendedId) -> IdReg {
        Self(id.as_raw() << Self::EXTENDED_SHIFT | Self::IDE_MASK)
    }

    fn from_register(reg: u32) -> IdReg {
        Self(reg & 0xFFFF_FFFE)
    }

    /// Sets the remote transmission (RTR) flag. This marks the identifier as
    /// being part of a remote frame.
    #[must_use = "returns a new IdReg without modifying `self`"]
    fn with_rtr(self, rtr: bool) -> IdReg {
        if rtr {
            Self(self.0 | Self::RTR_MASK)
        } else {
            Self(self.0 & !Self::RTR_MASK)
        }
    }

    /// Returns the identifier.
    fn to_id(self) -> Id {
        if self.is_extended() {
            Id::Extended(unsafe { ExtendedId::new_unchecked(self.0 >> Self::EXTENDED_SHIFT) })
        } else {
            Id::Standard(unsafe {
                StandardId::new_unchecked((self.0 >> Self::STANDARD_SHIFT) as u16)
            })
        }
    }

    /// Returns `true` if the identifier is an extended identifier.
    fn is_extended(self) -> bool {
        self.0 & Self::IDE_MASK != 0
    }

    /// Returns `true` if the identifier is a standard identifier.
    fn is_standard(self) -> bool {
        !self.is_extended()
    }

    /// Returns `true` if the identifer is part of a remote frame (RTR bit set).
    fn rtr(self) -> bool {
        self.0 & Self::RTR_MASK != 0
    }
}

/// `IdReg` is ordered by priority.
impl Ord for IdReg {
    fn cmp(&self, other: &Self) -> Ordering {
        // When the IDs match, data frames have priority over remote frames.
        let rtr = self.rtr().cmp(&other.rtr()).reverse();

        let id_a = self.to_id();
        let id_b = other.to_id();
        match (id_a, id_b) {
            (Id::Standard(a), Id::Standard(b)) => {
                // Lower IDs have priority over higher IDs.
                a.as_raw().cmp(&b.as_raw()).reverse().then(rtr)
            }
            (Id::Extended(a), Id::Extended(b)) => a.as_raw().cmp(&b.as_raw()).reverse().then(rtr),
            (Id::Standard(a), Id::Extended(b)) => {
                // Standard frames have priority over extended frames if their Base IDs match.
                a.as_raw()
                    .cmp(&b.standard_id().as_raw())
                    .reverse()
                    .then(Ordering::Greater)
            }
            (Id::Extended(a), Id::Standard(b)) => a
                .standard_id()
                .as_raw()
                .cmp(&b.as_raw())
                .reverse()
                .then(Ordering::Less),
        }
    }
}

impl PartialOrd for IdReg {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}


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

/// A CAN master
///
pub struct CAN<M> {
    reg: ral::can::Instance,
    _module: PhantomData<M>,
    source_clock: ccm::Frequency,
}

impl<M> CAN<M>
where
    M: Unsigned,
{

    fn new(source_clock: ccm::Frequency, reg: ral::can::Instance) -> Self {
        let mut can = CAN {
            reg,
            _module: PhantomData,
            source_clock,
        };
        can
    }
    
    fn set_bit_timing(&mut self, btr: u32) {
        // Mask of all non-reserved BTR bits, except the mode flags.

    }

    /// Returns a reference to the peripheral instance.
    ///
    /// This allows accessing HAL-specific data stored in the instance type.
    pub fn instance(&mut self) -> &mut ral::can::Instance {
        &mut self.reg
    }

    /// Disables the CAN interface and returns back the raw peripheral it was created from.
    ///
    /// The peripheral is disabled by setting `RESET` in `CAN_MCR`, which causes the peripheral to
    /// enter sleep mode.
    pub fn free(self) -> ral::can::Instance {
        self.reg
    }


    /// Configures the automatic wake-up feature.
    ///
    /// This is turned off by default.
    ///
    /// When turned on, an incoming frame will cause the peripheral to wake up from sleep and
    /// receive the frame. If enabled, [`Interrupt::Wakeup`] will also be triggered by the incoming
    /// frame.
    pub fn set_automatic_wakeup(&mut self, enabled: bool) {
      
    }

    /// Leaves initialization mode and enables the peripheral (non-blocking version).
    ///
    /// Usually, it is recommended to call [`CanConfig::enable`] instead. This method is only needed
    /// if you want non-blocking initialization.
    ///
    /// If this returns [`WouldBlock`][nb::Error::WouldBlock], the peripheral will enable itself
    /// in the background. The peripheral is enabled and ready to use when this method returns
    /// successfully.
    pub fn enable_non_blocking(&mut self) -> nb::Result<(), Infallible> {
        Ok(())
    }

    /// Puts the peripheral in a sleep mode to save power.
    ///
    /// While in sleep mode, an incoming CAN frame will trigger [`Interrupt::Wakeup`] if enabled.
    pub fn sleep(&mut self) {
       
    }

    /// Wakes up from sleep mode.
    ///
    /// Note that this will not trigger [`Interrupt::Wakeup`], only reception of an incoming CAN
    /// frame will cause that interrupt.
    pub fn wakeup(&mut self) {
       
    }

    /// Clears the pending flag of [`Interrupt::Sleep`].
    pub fn clear_sleep_interrupt(&self) {

    }

    /// Clears the pending flag of [`Interrupt::Wakeup`].
    pub fn clear_wakeup_interrupt(&self) {

    }

    /// Clears the "Request Completed" (RQCP) flag of a transmit mailbox.
    ///
    /// Returns the [`Mailbox`] whose flag was cleared. If no mailbox has the flag set, returns
    /// `None`.
    ///
    /// Once this function returns `None`, a pending [`Interrupt::TransmitMailboxEmpty`] is
    /// considered acknowledged.
    pub fn clear_request_completed_flag(&mut self) -> Option<Mailbox> {
        None
    }

    /// Clears a pending TX interrupt ([`Interrupt::TransmitMailboxEmpty`]).
    ///
    /// This does not return the mailboxes that have finished tranmission. If you need that
    /// information, call [`Can::clear_request_completed_flag`] instead.
    pub fn clear_tx_interrupt(&mut self) {
        while self.clear_request_completed_flag().is_some() {}
    }

    /// Puts a CAN frame in a free transmit mailbox for transmission on the bus.
    ///
    /// Frames are transmitted to the bus based on their priority (see [`FramePriority`]).
    /// Transmit order is preserved for frames with identical priority.
    ///
    /// If all transmit mailboxes are full, and `frame` has a higher priority than the
    /// lowest-priority message in the transmit mailboxes, transmission of the enqueued frame is
    /// cancelled and `frame` is enqueued instead. The frame that was replaced is returned as
    /// [`TransmitStatus::dequeued_frame`].
    pub fn transmit(&mut self, frame: &Frame) -> nb::Result<(), Infallible> {
       Ok(())
    }

    /// Returns `true` if no frame is pending for transmission.
    pub fn is_transmitter_idle(&self) -> bool {
        true
    }

    /// Attempts to abort the sending of a frame that is pending in a mailbox.
    ///
    /// If there is no frame in the provided mailbox, or its transmission succeeds before it can be
    /// aborted, this function has no effect and returns `false`.
    ///
    /// If there is a frame in the provided mailbox, and it is canceled successfully, this function
    /// returns `true`.
    pub fn abort(&mut self, mailbox: Mailbox) -> bool {
        // Safety: We have a `&mut self` and have unique access to the peripheral.
        true
    }

    /// Returns a received frame if available.
    ///
    /// This will first check FIFO 0 for a message or error. If none are available, FIFO 1 is
    /// checked.
    ///
    /// Returns `Err` when a frame was lost due to buffer overrun.
    pub fn receive(&mut self) -> nb::Result<(), OverrunError> {
        Ok(())
    }


}

/// Interface to the CAN transmitter part.
pub struct Tx<I> {
    _can: PhantomData<I>,
}

#[inline]
const fn ok_mask(idx: usize) -> u32 {
    0x02 << (8 * idx)
}

#[inline]
const fn abort_mask(idx: usize) -> u32 {
    0x80 << (8 * idx)
}

/// Identifies one of the two receive FIFOs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub enum Fifo {
    Fifo0 = 0,
    Fifo1 = 1,
}

/// Identifies one of the three transmit mailboxes.
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub enum Mailbox {
    /// Transmit mailbox 0
    Mailbox0 = 0,
    /// Transmit mailbox 1
    Mailbox1 = 1,
    /// Transmit mailbox 2
    Mailbox2 = 2,
}

/// Contains information about a frame enqueued for transmission via [`Can::transmit`] or
/// [`Tx::transmit`].
pub struct TransmitStatus {
    dequeued_frame: Option<Frame>,
    mailbox: Mailbox,
}

impl TransmitStatus {
    /// Returns the lower-priority frame that was dequeued to make space for the new frame.
    #[inline]
    pub fn dequeued_frame(&self) -> Option<&Frame> {
        self.dequeued_frame.as_ref()
    }

    /// Returns the [`Mailbox`] the frame was enqueued in.
    #[inline]
    pub fn mailbox(&self) -> Mailbox {
        self.mailbox
    }
}
