mod embedded_hal;
pub mod filter;
mod frame;
mod id;
mod interrupt;

pub use frame::{Data, Frame, FramePriority};
pub use id::{ExtendedId, Id, IdReg, StandardId};
use ral::{modify_reg, write_reg};

use crate::ccm;
use crate::iomuxc::consts::{Unsigned, U1, U2};
use crate::ral;

use core::cmp::Ord;
use core::convert::Infallible;
use core::marker::PhantomData;
use core::ops::Deref;

/// Error that indicates that an incoming message has been lost due to buffer overrun.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub struct OverrunError {
    _priv: (),
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
            CAN_CLK_PODF: ral::ccm::CSCMR2::CAN_CLK_PODF::RW::DIVIDE_1,
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

/// A CAN builder that can build a CAN peripheral
pub struct Builder<M> {
    _module: PhantomData<M>,
    reg: ral::can::Instance,
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
    /// is a configured FLEXCAN peripheral running at 24MHz.
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
    _mailbox_reader_index: u8,
}

const CAN1_ADDR: u32 = 0x401d0000;
const CAN2_ADDR: u32 = 0x401d4000;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MailboxData {
    pub code: u32,
    pub id: u32,
    pub data: [u8; 8],
    pub mailbox_index: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MailboxDataInfo {
    pub remote: bool,
    pub extended: bool,
    pub id: u32,
    pub length: u8,
    pub timestamp: u32,
}

impl From<&MailboxData> for MailboxDataInfo {
    fn from(d: &MailboxData) -> Self {
        let extended = d.code & (1_u32 << 21) == 1;

        Self {
            remote: d.code & (1_u32 << 20) == 1,
            extended: extended,
            id: (d.id & 0x1FFFFFFF_u32) >> (if extended { 0 } else { 18 }),
            length: ((d.code & 0xF0000_u32) >> 16) as u8,
            timestamp: d.code & 0xFFFF_u32,
        }
    }
}

impl<M> CAN<M>
where
    M: Unsigned,
{
    pub const NUMBER_FIFO_RX_MAILBOXES: u32 = 6;

    fn new(source_clock: ccm::Frequency, reg: ral::can::Instance) -> Self {
        let mut can = CAN {
            reg,
            _module: PhantomData,
            source_clock,
            _mailbox_reader_index: 0,
        };
        can.begin();
        can
    }

    pub fn instance(&mut self) -> &mut ral::can::Instance {
        &mut self.reg
    }

    pub fn print_registers(&self) {
        log::info!("MCR: {:X}", ral::read_reg!(ral::can, self.reg, MCR));
        log::info!("CTRL1: {:X}", ral::read_reg!(ral::can, self.reg, CTRL1));
        log::info!("CTRL2: {:X}", ral::read_reg!(ral::can, self.reg, CTRL2));
    }

    pub fn begin(&mut self) {
        log::info!("CCM CSCMR2 CAN_CLK_SEL: {:X}", unsafe {
            ral::read_reg!(ral::ccm, CCM, CSCMR2, CAN_CLK_SEL)
        });

        self.set_ccm_ccg();
        self.set_tx();
        self.set_rx();

        ral::modify_reg!(ral::can, self.reg, MCR, MDIS: MDIS_0);

        self.print_registers();

        self.enter_freeze_mode();

        ral::modify_reg!(ral::can, self.reg, CTRL1, LOM: LOM_1);
        ral::modify_reg!(ral::can, self.reg, MCR, FRZ: FRZ_1);
        while ral::read_reg!(ral::can, self.reg, MCR, LPMACK == LPMACK_1) {}

        self.soft_reset();

        while ral::read_reg!(ral::can, self.reg, MCR, FRZACK != FRZACK_1) {}

        ral::modify_reg!(
            ral::can,
            self.reg,
            MCR,
            SRXDIS: SRXDIS_1,
            IRMQ: IRMQ_1,
            AEN: AEN_1,
            LPRIOEN: LPRIOEN_1,
            SLFWAK: SLFWAK_1,
            WAKSRC: WAKSRC_1,
            WRNEN: WRNEN_1,
            WAKMSK: WAKMSK_1
        );

        ral::modify_reg!(ral::can, self.reg, MCR, |reg| reg & !0x8800);

        ral::modify_reg!(
            ral::can,
            self.reg,
            CTRL2,
            RRS: RRS_1,
            EACEN: EACEN_1,
            MRP: MRP_1
        );

        self.print_registers();

        // self.disable_fifo();
        self.exit_freeze_mode();
    }

    pub fn instance_number(&self) -> usize {
        M::USIZE
    }

    pub fn is_can1(&self) -> bool {
        M::USIZE == 1
    }

    pub fn is_can2(&self) -> bool {
        M::USIZE == 2
    }

    pub fn base_address(&self) -> u32 {
        match self.is_can1() {
            true => CAN1_ADDR,
            false => CAN2_ADDR,
        }
    }

    pub fn free(self) -> ral::can::Instance {
        self.reg
    }

    fn soft_reset(&mut self) {
        ral::modify_reg!(ral::can, self.reg, MCR, SOFTRST: SOFTRST_1);
        while ral::read_reg!(ral::can, self.reg, MCR, SOFTRST == SOFTRST_1) {}
    }

    fn enter_freeze_mode(&mut self) {
        ral::modify_reg!(ral::can, self.reg, MCR, FRZ: FRZ_1);
        ral::modify_reg!(ral::can, self.reg, MCR, HALT: HALT_1);
        while ral::read_reg!(ral::can, self.reg, MCR, FRZACK != FRZACK_1) {}
    }

    fn exit_freeze_mode(&mut self) {
        ral::modify_reg!(ral::can, self.reg, MCR, HALT: HALT_0);
        while ral::read_reg!(ral::can, self.reg, MCR, FRZACK != FRZACK_0) {}
    }

    fn set_mrp(&mut self, mrp: bool) {
        ral::modify_reg!(ral::can, self.reg, CTRL2, MRP: mrp as u32)
    }

    fn set_rrs(&mut self, rrs: bool) {
        ral::modify_reg!(ral::can, self.reg, CTRL2, RRS: rrs as u32)
    }

    fn set_ccm_ccg(&mut self) {
        match self.instance_number() {
            1 => {
                unsafe {
                    modify_reg!(
                        ral::ccm,
                        CCM,
                        CCGR0,
                        |reg| reg | 0x3C000
                    )
                };
            }
            2 => {
                unsafe {
                    modify_reg!(
                        ral::ccm,
                        CCM,
                        CCGR0,
                        |reg| reg | 0x3C0000
                    )
                };
            }
            _ => {}
        }
    }

    fn set_tx(&mut self) {
        match self.instance_number() {
            1 => {
                unsafe {
                    modify_reg!(
                        ral::iomuxc,
                        IOMUXC,
                        SW_MUX_CTL_PAD_GPIO_AD_B1_08,
                        MUX_MODE: ALT2,
                        SION: ENABLED
                    )
                };
                unsafe {
                    write_reg!(
                        ral::iomuxc,
                        IOMUXC,
                        SW_PAD_CTL_PAD_GPIO_AD_B0_08,
                        0x10B0_u32
                    )
                };
            }
            2 => {
                unsafe {
                    modify_reg!(
                        ral::iomuxc,
                        IOMUXC,
                        SW_MUX_CTL_PAD_GPIO_AD_B0_02,
                        MUX_MODE: ALT0,
                        SION: ENABLED
                    )
                };
                unsafe {
                    write_reg!(
                        ral::iomuxc,
                        IOMUXC,
                        SW_PAD_CTL_PAD_GPIO_AD_B0_02,
                        0x10B0_u32
                    )
                };
            }
            _ => {}
        }
    }

    fn set_rx(&mut self) {
        match self.instance_number() {
            1 => {
                unsafe {
                    modify_reg!(
                        ral::iomuxc,
                        IOMUXC,
                        FLEXCAN1_RX_SELECT_INPUT,
                        DAISY: GPIO_AD_B1_09_ALT2
                    )
                };
                unsafe {
                    modify_reg!(
                        ral::iomuxc,
                        IOMUXC,
                        SW_MUX_CTL_PAD_GPIO_AD_B1_09,
                        MUX_MODE: ALT2,
                        SION: ENABLED
                    )
                };
                unsafe {
                    write_reg!(
                        ral::iomuxc,
                        IOMUXC,
                        SW_PAD_CTL_PAD_GPIO_AD_B1_09,
                        0x10B0_u32
                    )
                };
            }
            2 => {
                unsafe {
                    modify_reg!(
                        ral::iomuxc,
                        IOMUXC,
                        FLEXCAN2_RX_SELECT_INPUT,
                        DAISY: GPIO_AD_B0_03_ALT0
                    )
                };
                unsafe {
                    modify_reg!(
                        ral::iomuxc,
                        IOMUXC,
                        SW_MUX_CTL_PAD_GPIO_AD_B0_03,
                        MUX_MODE: ALT0,
                        SION: ENABLED
                    )
                };
                unsafe {
                    write_reg!(
                        ral::iomuxc,
                        IOMUXC,
                        SW_PAD_CTL_PAD_GPIO_AD_B0_03,
                        0x10B0_u32
                    )
                };
            }
            _ => {}
        }
    }

    fn set_baud_rate(&mut self) {}

    fn set_max_mailbox(&mut self, last: u8) {
        let last = match last {
            _l if last >= 64 => 63,
            _l if last <= 1 => 0,
            _ => last - 1,
        };
        self.enter_freeze_mode();
        let fifo_cleared = self.fifo_enabled();
        self.disable_fifo();
        self.write_iflag(self.read_iflag());
        ral::modify_reg!(ral::can, self.reg, MCR, MAXMB: last as u32);
        if fifo_cleared {
            self.enable_fifo(true);
        }
        self.exit_freeze_mode();
    }

    fn get_max_mailbox(&self) -> u8 {
        ral::read_reg!(ral::can, self.reg, MCR, MAXMB) as u8
    }

    fn write_iflag(&mut self, value: u64) {
        write_reg!(ral::can, self.reg, IFLAG1, value as u32);
        write_reg!(ral::can, self.reg, IFLAG2, (value >> 32) as u32);
    }

    fn write_iflag_bit(&mut self, mailbox_number: u8) {
        if mailbox_number < 32 {
            modify_reg!(ral::can, self.reg, IFLAG1, |reg| reg
                | 1_u32 << mailbox_number)
        } else {
            modify_reg!(ral::can, self.reg, IFLAG2, |reg| reg
                | 1_u32 << (mailbox_number - 32))
        }
    }

    fn read_iflag(&self) -> u64 {
        (ral::read_reg!(ral::can, self.reg, IFLAG2) as u64) << 32
            | ral::read_reg!(ral::can, self.reg, IFLAG1) as u64
    }

    fn read_imask(&self) -> u64 {
        (ral::read_reg!(ral::can, self.reg, IMASK2) as u64) << 32
            | ral::read_reg!(ral::can, self.reg, IMASK1) as u64
    }

    fn write_imask(&mut self, value: u64) {
        write_reg!(ral::can, self.reg, IMASK1, value as u32);
        write_reg!(ral::can, self.reg, IMASK2, (value >> 32) as u32);
    }

    fn enable_fifo(&mut self, enabled: bool) {
        let frz_flag_negate = ral::read_reg!(ral::can, self.reg, MCR, FRZACK == FRZACK_1);

        self.enter_freeze_mode();

        modify_reg!(ral::can, self.reg, MCR, RFEN: RFEN_0);

        self.write_imask(0);

        for i in 0..self.get_max_mailbox() {
            self._write_mailbox(i, 0, 0, 0, 0);
        }

        write_reg!(ral::can, self.reg, RXMGMASK, 0);
        write_reg!(ral::can, self.reg, RXFGMASK, 0);

        self.write_iflag(self.read_iflag());

        if enabled {
            modify_reg!(ral::can, self.reg, MCR, RFEN: RFEN_1);
        } else {
            let max_mailbox = self.get_max_mailbox();
            for i in 0..max_mailbox {
                if 1 < max_mailbox / 2 {
                    self._write_mailbox(i, 0, 0, 0, 0);
                } else {
                    self._write_mailbox(i, 0, 0, 0, 0);
                }
            }
        }

        if frz_flag_negate {
            self.exit_freeze_mode();
        }
    }

    fn disable_fifo(&mut self) {
        self.enable_fifo(false);
    }

    fn fifo_enabled(&self) -> bool {
        ral::read_reg!(ral::can, self.reg, MCR, RFEN == RFEN_1)
    }

    fn mailbox_offset(&self) -> u8 {
        if self.fifo_enabled() {
            let filter_space = Self::NUMBER_FIFO_RX_MAILBOXES
                + (ral::read_reg!(ral::can, self.reg, CTRL2, RFFN) + 1) * 2;
            let max_mailboxes = ral::read_reg!(ral::can, self.reg, MCR, MAXMB);
            if max_mailboxes < filter_space {
                0
            } else {
                (max_mailboxes - filter_space) as u8
            }
        } else {
            0
        }
    }

    fn read_fifo(&self) -> Option<()> {
        // if FIFO enabled and interrupt not enabled
        if !self.fifo_enabled() {
            return None;
        };
        // FIFO interrupt enabled, polling blocked
        if (ral::read_reg!(ral::can, self.reg, IMASK1) & (0x00000020 as u32)) != 0 {
            return None;
        }
        // message available
        if (ral::read_reg!(ral::can, self.reg, IFLAG1) & (0x00000020 as u32)) != 0 {
            return None;
        }
        Some(())
    }

    fn _mailbox_index_to_address(&self, mailbox_index: u8) -> u32 {
        self.base_address() + 0x80_u32 + (mailbox_index as u32 * 0x10_u32)
    }

    fn _read_mailbox(&self, mailbox_index: u8) -> Option<MailboxData> {
        let mailbox_addr = self._mailbox_index_to_address(mailbox_index);
        log::info!(
            "mailbox_addr: {:X}, imask: {:X}",
            &mailbox_addr,
            self.read_imask()
        );

        if (self.read_imask() & (1_u64 << mailbox_index)) != 0 {
            log::info!("Int Enablesd! {:?}", self.read_imask());
            return None;
        }

        // let code = unsafe { core::ptr::read_volatile(mailbox_addr as *const u32) };
        // let c = (code * 0x0F000000_u32) >> 24;

        // log::info!("Code: {:?}", &code);

        // match ((c & 0x0F000000_u32) >> 24) as u8 {
        //     // return None from a transmit mailbox
        //     c if c >> 3 != 0 => None,
        //     // full or overrun
        //     c if c == 0b0010_u8 || c == 0b0110_u8 => {
        //         let id = unsafe { core::ptr::read_volatile((mailbox_addr + 1_u32) as *const u32) };
        //         let data0 =
        //             unsafe { core::ptr::read_volatile((mailbox_addr + 2_u32) as *const u32) };
        //         let data1 =
        //             unsafe { core::ptr::read_volatile((mailbox_addr + 3_u32) as *const u32) };

        //         let mut data: [u8; 8] = [0, 0, 0, 0, 0, 0, 0, 0];
        //         for i in 0..4 {
        //             data[3 - i] = (data0 >> (8 * i)) as u8;
        //         }
        //         for i in 0..4 {
        //             data[7 - i] = (data1 >> (8 * i)) as u8;
        //         }

        //         Some(MailboxData {
        //             code: code,
        //             id: id,
        //             data: data,
        //             mailbox_index: mailbox_index,
        //         })
        //     }
        //     _ => None,
        // }
        None
    }

    fn _write_mailbox(&self, mailbox_index: u8, code: u32, id: u32, data0: u32, data1: u32) {
        let mailbox_addr = self._mailbox_index_to_address(mailbox_index);
        unsafe { core::ptr::write_volatile((mailbox_addr + 0_u32) as *mut u32, code) };
        unsafe { core::ptr::write_volatile((mailbox_addr + 1_u32) as *mut u32, id) };
        unsafe { core::ptr::write_volatile((mailbox_addr + 2_u32) as *mut u32, data0) };
        unsafe { core::ptr::write_volatile((mailbox_addr + 3_u32) as *mut u32, data1) };
    }

    pub fn read_mailbox(&mut self) -> Option<()> {
        let mut iflag: u64;
        let mut cycle_limit: u8 = 3;
        let offset = self.mailbox_offset();
        let max_mailbox = self.get_max_mailbox();
        log::info!("offset: {:?}, max_mailbox: {:?}", offset, max_mailbox);
        let base = unsafe { core::ptr::read_volatile(self.base_address() as *const u32) };
        log::info!("base: {:X}", base);
        while self._mailbox_reader_index <= max_mailbox {
            iflag = self.read_iflag();
            if iflag != 0 && (self._mailbox_reader_index >= (64 - iflag.leading_zeros() as u8)) {
                /* break from MSB's if unset, add 1 to prevent undefined behaviour in clz for 0 check */
                self._mailbox_reader_index = self.mailbox_offset();
                cycle_limit -= 1;
                if cycle_limit == 0 {
                    return None;
                }
            }
            if self.fifo_enabled() {
                /* FIFO is enabled, get only remaining RX (if any) */
                if self._mailbox_reader_index < offset {
                    /* go back to position end of fifo+filter region */
                    self._mailbox_reader_index = offset;
                }
            }
            if self._mailbox_reader_index >= max_mailbox {
                self._mailbox_reader_index = self.mailbox_offset();
                cycle_limit -= 1;
                if cycle_limit == 0 {
                    return None;
                }
            }
            if (self.read_imask() & (1_u64 << self._mailbox_reader_index)) != 0 {
                self._mailbox_reader_index += 1;
                continue; /* don't read interrupt enabled mailboxes */
            }

            if let Some(mailbox_data) = self._read_mailbox(self._mailbox_reader_index) {
                self.write_iflag_bit(self._mailbox_reader_index);
                log::info!(
                    "RX Data: {:?}, {:?}",
                    &mailbox_data,
                    MailboxDataInfo::from(&mailbox_data)
                );
            }
            self._mailbox_reader_index += 1;
        }
        None
    }

    /// Configures the automatic wake-up feature.
    ///
    /// This is turned off by default.
    ///
    /// When turned on, an incoming frame will cause the peripheral to wake up from sleep and
    /// receive the frame. If enabled, [`Interrupt::Wakeup`] will also be triggered by the incoming
    /// frame.
    pub fn set_automatic_wakeup(&mut self, enabled: bool) {}

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
    pub fn sleep(&mut self) {}

    /// Wakes up from sleep mode.
    ///
    /// Note that this will not trigger [`Interrupt::Wakeup`], only reception of an incoming CAN
    /// frame will cause that interrupt.
    pub fn wakeup(&mut self) {}

    /// Clears the pending flag of [`Interrupt::Sleep`].
    pub fn clear_sleep_interrupt(&self) {}

    /// Clears the pending flag of [`Interrupt::Wakeup`].
    pub fn clear_wakeup_interrupt(&self) {}

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
