mod embedded_hal;
pub mod filter;
mod frame;
mod id;
mod interrupt;

pub use frame::{Data, Frame, FramePriority};
pub use id::{ExtendedId, Id, IdReg, StandardId};
use ral::{modify_reg, read_reg, write_reg};

use crate::ccm;
use crate::iomuxc::consts::{Unsigned, U1, U2};
use crate::ral;

use core::convert::Infallible;
use core::marker::PhantomData;

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
    pub mailbox_number: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MailboxDataInfo {
    pub remote: bool,
    pub extended: bool,
    pub id: Id,
    pub length: u8,
    pub timestamp: u32,
}

impl From<&MailboxData> for MailboxDataInfo {
    fn from(d: &MailboxData) -> Self {
        let extended = d.code & (1_u32 << 21) == 1;
        let id = (d.id & 0x1FFFFFFF_u32) >> (if extended { 0 } else { 18 });
        let id: Id = if extended {
            ExtendedId::new(id).unwrap().into()
        } else {
            StandardId::new(id as u16).unwrap().into()
        };
        Self {
            remote: d.code & (1_u32 << 20) == 1,
            extended,
            id: id,
            length: ((d.code & 0xF0000_u32) >> 16) as u8,
            timestamp: d.code & 0xFFFF_u32,
        }
    }
}

impl From<&Frame> for MailboxData {
    fn from(f: &Frame) -> Self {
        Self {
            code: 0x00,
            id: f.id.to_id().as_raw(),
            data: f.data.bytes.into(),
            mailbox_number: 0,
        }
    }
}

const FLEXCAN_MB_CS_CODE_MASK: u32 = 0x0F000000;

#[repr(u8)]
pub enum FlexCanMailboxCSCode {
    RxInactive = 0b0000,
    RxEmpty = 0b0100,
    RxFull = 0b0010,
    RxOverrun = 0b0110,
    RxBusy = 0b0001,

    TxInactive = 0b1000,
    TxAbort = 0b1001,
    TxOnce = 0b1100,
}

#[inline]
fn to_flexcan_mb_cs_code(status: u8) -> u32 {
    ((status as u32) & 0x0000000F) << 24
}

#[inline]
fn from_flexcan_mb_cs_code(code: u32) -> u8 {
    ((code & FLEXCAN_MB_CS_CODE_MASK) >> 24) as u8
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

    fn while_frozen<F: FnMut(&mut Self) -> R, R>(&mut self, mut act: F) -> R {
        let frz_flag_negate = ral::read_reg!(ral::can, self.reg, MCR, FRZACK == FRZACK_0);
        self.enter_freeze_mode();
        let res = act(self);
        if frz_flag_negate {
            self.exit_freeze_mode();
        }
        res
    }

    pub fn begin(&mut self) {
        self.set_ccm_ccg();
        self.set_tx();
        self.set_rx();

        ral::modify_reg!(ral::can, self.reg, MCR, MDIS: MDIS_0);

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

        self.disable_fifo();
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
                unsafe { modify_reg!(ral::ccm, CCM, CCGR0, |reg| reg | 0x3C000) };
            }
            2 => {
                unsafe { modify_reg!(ral::ccm, CCM, CCGR0, |reg| reg | 0x3C0000) };
            }
            u => {
                log::error!("Invalid CAN instance (set_ccm_ccg): {:?}", u);
            }
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
                        SW_PAD_CTL_PAD_GPIO_AD_B1_08,
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
            u => {
                log::error!("Invalid CAN instance (set_tx): {:?}", u);
            }
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
            u => {
                log::error!("Invalid CAN instance (set_rx): {:?}", u);
            }
        }
    }

    fn get_clock(&self) -> u32 {
        self.source_clock.0
    }

    fn result_to_bit_table(&self, result: u8) -> Option<[u32; 3]> {
        match result {
            0 => Some([0, 0, 1]),
            1 => Some([1, 0, 1]),
            2 => Some([1, 1, 1]),
            3 => Some([2, 1, 1]),
            4 => Some([2, 2, 1]),
            5 => Some([2, 3, 1]),
            6 => Some([2, 3, 2]),
            7 => Some([2, 4, 2]),
            8 => Some([2, 5, 2]),
            9 => Some([2, 5, 3]),
            10 => Some([2, 6, 3]),
            11 => Some([2, 7, 3]),
            12 => Some([2, 7, 4]),
            13 => Some([3, 7, 4]),
            14 => Some([3, 7, 5]),
            15 => Some([4, 7, 5]),
            16 => Some([4, 7, 6]),
            17 => Some([5, 7, 6]),
            18 => Some([6, 7, 6]),
            19 => Some([6, 7, 7]),
            20 => Some([7, 7, 7]),
            _ => None,
        }
    }

    pub fn set_baud_rate(&mut self, baud: u32) {
        let clock_freq = 24_000_000;

        let mut divisor = 0;
        let mut best_divisor: u32 = 0;
        let mut result: u32 = clock_freq / baud / (divisor + 1);
        let mut error: i16 = (baud - (clock_freq / (result * (divisor + 1)))) as i16;
        let mut best_error = error;

        self.while_frozen(|this| {
            while result > 5 {
                divisor += 1;
                result = clock_freq / baud / (divisor + 1);
                if result <= 25 {
                    error = (baud - (clock_freq / (result * (divisor + 1)))) as i16;
                    if error < 0 {
                        error = -1 * error;
                    }
                    if error < best_error {
                        best_error = error;
                        best_divisor = divisor;
                    }
                    if (error == best_error) && (result > 11) && (result < 19) {
                        best_error = error;
                        best_divisor = divisor;
                    }
                }
            }

            divisor = best_divisor;
            result = clock_freq / baud / (divisor + 1);

            if !(result < 5) || (result > 25) || (best_error > 300) {
                result -= 5;

                match this.result_to_bit_table(result as u8) {
                    Some(t) => {
                        modify_reg!(
                            ral::can,
                            this.reg,
                            CTRL1,
                            PROPSEG: t[0],
                            RJW: 1,
                            PSEG1: t[1],
                            PSEG2: t[2],
                            ERRMSK: ERRMSK_1,
                            LOM: LOM_0,
                            PRESDIV: divisor)
                    }
                    _ => {}
                }
            }
        });
    }

    pub fn set_max_mailbox(&mut self, last: u8) {
        let last = match last {
            _l if last >= 64 => 63,
            _l if last <= 1 => 0,
            _ => last - 1,
        };
        self.while_frozen(|this| {
            let fifo_cleared = this.fifo_enabled();
            this.disable_fifo();
            this.write_iflag(this.read_iflag());
            ral::modify_reg!(ral::can, this.reg, MCR, MAXMB: last as u32);
            if fifo_cleared {
                this.enable_fifo(true);
            }
        });
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

    fn write_imask_bit(&mut self, mailbox_number: u8, value: bool) {
        if mailbox_number < 32 {
            modify_reg!(ral::can, self.reg, IMASK1, |reg| reg
                | (value as u32) << mailbox_number)
        } else {
            modify_reg!(ral::can, self.reg, IMASK2, |reg| reg
                | (value as u32) << (mailbox_number - 32))
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

    pub fn enable_fifo(&mut self, enabled: bool) {
        self.while_frozen(|this| {
            modify_reg!(ral::can, this.reg, MCR, RFEN: RFEN_0);

            this.write_imask(0);

            for i in 0..this.get_max_mailbox() {
                this.write_mailbox(i, Some(0), Some(0), Some(0), Some(0));
                this.write_mailbox_rximr(i, Some(0x00));
            }

            write_reg!(ral::can, this.reg, RXMGMASK, 0);
            write_reg!(ral::can, this.reg, RXFGMASK, 0);

            this.write_iflag(this.read_iflag());

            let max_mailbox = this.get_max_mailbox();
            if enabled {
                modify_reg!(ral::can, this.reg, MCR, RFEN: RFEN_1);
                while ral::read_reg!(ral::can, this.reg, MCR, RFEN != RFEN_1) {}
                for i in this.mailbox_offset()..max_mailbox {
                    this.write_mailbox(
                        i,
                        Some(to_flexcan_mb_cs_code(
                            FlexCanMailboxCSCode::TxInactive as u8,
                        )),
                        None,
                        None,
                        None,
                    );
                    this.enable_mailbox_interrupt(i, true);
                }
            } else {
                for i in 0..max_mailbox {
                    if i < max_mailbox / 2 {
                        let code = to_flexcan_mb_cs_code(FlexCanMailboxCSCode::RxEmpty as u8)
                            | 0x00400000
                            | {
                                if i < max_mailbox / 4 {
                                    0
                                } else {
                                    0x00200000
                                }
                            };
                        this.write_mailbox(i, Some(code), None, None, None);
                        let eacen = read_reg!(ral::can, this.reg, CTRL2, EACEN == EACEN_1);
                        let rximr = 0_32 | {
                            if eacen {
                                1_u32 << 30
                            } else {
                                0
                            }
                        };
                        this.write_mailbox_rximr(i, Some(rximr));
                    } else {
                        this.write_mailbox(
                            i,
                            Some(to_flexcan_mb_cs_code(
                                FlexCanMailboxCSCode::TxInactive as u8,
                            )),
                            Some(0),
                            Some(0),
                            Some(0),
                        );
                        this.enable_mailbox_interrupt(i, true);
                    }
                }
            }
        })
    }

    fn disable_fifo(&mut self) {
        self.enable_fifo(false);
    }

    pub fn enable_fifo_interrupt(&mut self, enabled: bool) {
        /* FIFO must be enabled first */
        if !self.fifo_enabled() {
            return;
        };
        /* FIFO interrupts already enabled */
        if (ral::read_reg!(ral::can, self.reg, IMASK1, BUFLM) & 0x00000020) != 0 {
            return;
        };
        /* disable FIFO interrupt flags */
        modify_reg!(ral::can, self.reg, IMASK1, |reg| reg & !0xFF);
        /* enable FIFO interrupt */
        if enabled {
            modify_reg!(ral::can, self.reg, IMASK1, BUFLM: 0x00000020);
        }
    }

    fn fifo_enabled(&self) -> bool {
        ral::read_reg!(ral::can, self.reg, MCR, RFEN == RFEN_1)
    }

    fn mailbox_offset(&self) -> u8 {
        if self.fifo_enabled() {
            let max_mailbox = self.get_max_mailbox() as u32;
            let num_rx_fifo_filters = (read_reg!(ral::can, self.reg, CTRL2, RFFN) + 1) * 2;
            let remaining_mailboxes = max_mailbox - 6_u32 - num_rx_fifo_filters;
            /* return offset MB position after FIFO area */
            if max_mailbox < max_mailbox - remaining_mailboxes {
                max_mailbox as u8
            } else {
                (max_mailbox - remaining_mailboxes) as u8
            }
        } else {
            /* return offset 0 since FIFO is disabled */
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

    fn mailbox_number_to_address(&self, mailbox_number: u8) -> u32 {
        self.base_address() + 0x80_u32 + (mailbox_number as u32 * 0x10_u32)
    }

    fn mailbox_number_to_rximr_address(&self, mailbox_number: u8) -> u32 {
        self.base_address() + 0x880_u32 + (mailbox_number as u32 * 0x4_u32)
    }

    fn read_mailbox_code(&mut self, mailbox_number: u8) -> Option<u8> {
        let mailbox_addr = self.mailbox_number_to_address(mailbox_number);
        let code = unsafe { core::ptr::read_volatile(mailbox_addr as *const u32) };
        Some(((code & 0x0F000000_u32) >> 24) as u8)
    }

    fn read_mailbox(&mut self, mailbox_number: u8) -> Option<MailboxData> {
        let mailbox_addr = self.mailbox_number_to_address(mailbox_number);

        if (self.read_imask() & (1_u64 << mailbox_number)) != 0 {
            return None;
        }

        let code = unsafe { core::ptr::read_volatile(mailbox_addr as *const u32) };
        let c = ((code & 0x0F000000_u32) >> 24) as u8;

        match c {
            // return None from a transmit mailbox
            c if c >> 3 != 0 => {
                // log::info!("transmit code!");
                None
            }
            // full or overrun
            c if (c == FlexCanMailboxCSCode::RxFull as u8)
                | (c == FlexCanMailboxCSCode::RxOverrun as u8) =>
            {
                let id =
                    unsafe { core::ptr::read_volatile((mailbox_addr + 0x4_u32) as *const u32) };
                let data0 =
                    unsafe { core::ptr::read_volatile((mailbox_addr + 0x8_u32) as *const u32) };
                let data1 =
                    unsafe { core::ptr::read_volatile((mailbox_addr + 0xC_u32) as *const u32) };

                let mut data: [u8; 8] = [0, 0, 0, 0, 0, 0, 0, 0];
                for i in 0..4 {
                    data[3 - i] = (data0 >> (8 * i)) as u8;
                }
                for i in 0..4 {
                    data[7 - i] = (data1 >> (8 * i)) as u8;
                }

                self.write_mailbox(
                    mailbox_number,
                    Some(to_flexcan_mb_cs_code(FlexCanMailboxCSCode::RxEmpty as u8)),
                    None,
                    None,
                    None,
                );
                read_reg!(ral::can, self.reg, TIMER);
                self.write_iflag_bit(mailbox_number);

                Some(MailboxData {
                    code,
                    id,
                    data,
                    mailbox_number,
                })
            }
            _ => None,
        }
    }

    fn write_mailbox(
        &self,
        mailbox_number: u8,
        code: Option<u32>,
        id: Option<u32>,
        word0: Option<u32>,
        word1: Option<u32>,
    ) {
        let mailbox_addr = self.mailbox_number_to_address(mailbox_number);
        if let Some(code) = code {
            unsafe { core::ptr::write_volatile((mailbox_addr + 0_u32) as *mut u32, code) };
        }
        if let Some(id) = id {
            unsafe { core::ptr::write_volatile((mailbox_addr + 0x4_u32) as *mut u32, id) };
        }
        if let Some(word0) = word0 {
            unsafe { core::ptr::write_volatile((mailbox_addr + 0x8_u32) as *mut u32, word0) };
        }
        if let Some(word1) = word1 {
            unsafe { core::ptr::write_volatile((mailbox_addr + 0xC_u32) as *mut u32, word1) };
        }
    }

    fn _write_mailbox_2(
        &self,
        mailbox_number: u8,
        code: Option<u32>,
        id: Option<u32>,
        data: Option<[u8; 8]>,
    ) {
        let mailbox_addr = self.mailbox_number_to_address(mailbox_number);
        if let Some(code) = code {
            unsafe { core::ptr::write_volatile((mailbox_addr + 0_u32) as *mut u32, code) };
        }
        if let Some(id) = id {
            unsafe { core::ptr::write_volatile((mailbox_addr + 0x4_u32) as *mut u32, id) };
        }
        if let Some(data) = data {
            let word0 = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
            let word1 = u32::from_be_bytes([data[4], data[5], data[6], data[7]]);
            unsafe { core::ptr::write_volatile((mailbox_addr + 0x8_u32) as *mut u32, word0) };
            unsafe { core::ptr::write_volatile((mailbox_addr + 0xC_u32) as *mut u32, word1) };
        }
    }

    fn write_tx_mailbox(&mut self, mailbox_number: u8, id: u32, word0: u32, word1: u32) {
        self.write_iflag_bit(mailbox_number);
        let mut code: u32 = 0x00;
        self.write_mailbox(
            mailbox_number,
            Some(to_flexcan_mb_cs_code(
                FlexCanMailboxCSCode::TxInactive as u8,
            )),
            None,
            None,
            None,
        );
        self.write_mailbox(
            mailbox_number,
            None,
            Some((id & 0x000007FF) << 18),
            Some(word0),
            Some(word1),
        );
        code |= 8 << 16 | to_flexcan_mb_cs_code(FlexCanMailboxCSCode::TxOnce as u8);
        self.write_mailbox(mailbox_number, Some(code), None, None, None);
    }

    fn write_tx_mailbox_2(&mut self, mailbox_number: u8, id: u32, data: [u8; 8]) {
        self.write_iflag_bit(mailbox_number);
        let mut code: u32 = 0x00;
        self.write_mailbox(
            mailbox_number,
            Some(to_flexcan_mb_cs_code(
                FlexCanMailboxCSCode::TxInactive as u8,
            )),
            None,
            None,
            None,
        );
        self._write_mailbox_2(mailbox_number, None, Some(id), Some(data));
        code |= 8 << 16 | to_flexcan_mb_cs_code(FlexCanMailboxCSCode::TxOnce as u8);
        self.write_mailbox(mailbox_number, Some(code), None, None, None);
    }

    fn write_mailbox_rximr(&self, mailbox_number: u8, rximr: Option<u32>) {
        let mailbox_rximr_addr = self.mailbox_number_to_rximr_address(mailbox_number);
        if let Some(rximr) = rximr {
            unsafe { core::ptr::write_volatile((mailbox_rximr_addr) as *mut u32, rximr) };
        }
    }

    fn enable_mailbox_interrupt(&mut self, mailbox_number: u8, enabled: bool) {
        /* mailbox not available */
        if mailbox_number < self.mailbox_offset() {
            return;
        }
        if enabled {
            /* enable mailbox interrupt */
            self.write_imask_bit(mailbox_number, true);
            return;
        } else {
            match self.read_mailbox(mailbox_number) {
                Some(d) => {
                    if (d.code & 0x0F000000) >> 3 != 0 {
                        /* transmit interrupt keeper */
                        self.write_imask_bit(mailbox_number, true);
                        return;
                    }
                }
                _ => {}
            }
        }
        /* disable mailbox interrupt */
        self.write_imask_bit(mailbox_number, false);
        return;
    }

    pub fn read_mailboxes(&mut self) -> Option<()> {
        let mut iflag: u64;
        let mut cycle_limit: u8 = 3;
        let offset = self.mailbox_offset();
        let max_mailbox = self.get_max_mailbox();
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
            match self.read_mailbox(self._mailbox_reader_index) {
                Some(mailbox_data) => {
                    log::info!(
                        "RX Data: {:?}, {:?}",
                        &mailbox_data,
                        MailboxDataInfo::from(&mailbox_data)
                    );
                }
                _ => {}
            }
            self._mailbox_reader_index += 1;
        }
        None
    }

    pub fn handle_interrupt(&mut self) -> Option<MailboxData> {
        let imask = self.read_imask();
        let iflag = self.read_iflag();

        /* if DMA is disabled, ONLY THEN can you handle FIFO in ISR */
        if self.fifo_enabled() & (imask & 0x00000020 != 0) & (iflag & 0x00000020 != 0) {
            if let Some(mailbox_data) = self.read_mailbox(0) {
                self.write_iflag_bit(5);
                if iflag & 0x00000040 != 0 {
                    self.write_iflag_bit(6);
                }
                if iflag & 0x00000080 != 0 {
                    self.write_iflag_bit(7);
                }
                return Some(mailbox_data);
            }
        }

        None
    }

    pub fn transmit(&mut self, frame: &Frame) -> nb::Result<(), Infallible> {
        for i in self.mailbox_offset()..self.get_max_mailbox() {
            if let Some(code) = self.read_mailbox_code(i) {
                if code == FlexCanMailboxCSCode::TxInactive as u8 {
                    let id = frame.id.to_id().as_raw();
                    self.write_tx_mailbox_2(i, id, frame.data.bytes);
                    return Ok(());
                }
            }
        }
        Ok(())
    }

    pub fn write(&mut self, id: u32, word0: u32, word1: u32) -> nb::Result<(), Infallible> {
        for i in self.mailbox_offset()..self.get_max_mailbox() {
            if let Some(code) = self.read_mailbox_code(i) {
                if code == FlexCanMailboxCSCode::TxInactive as u8 {
                    self.write_tx_mailbox(i, id, word0, word1);
                    return Ok(());
                }
            }
        }
        Ok(())
    }
}

/// Interface to the CAN transmitter part.
pub struct Tx<I> {
    _can: PhantomData<I>,
}
