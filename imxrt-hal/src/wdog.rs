#[allow(unused_imports)]
use embedded_hal::watchdog;

use crate::ccm;
use crate::iomuxc::consts::Unsigned;
use crate::ral;
use crate::ral::wdog::{self, Instance};
use core::marker::PhantomData;

/// The WDOG, not yet enabled.
pub struct Unclocked<M> {
    reg: Instance,
    _module: PhantomData<M>,
}

impl<M> Unclocked<M>
where
    M: Unsigned,
{
    pub(crate) fn new(reg: Instance) -> Self {
        Unclocked {
            reg,
            _module: PhantomData,
        }
    }

    /// Enable clocks to all SPI modules, returning a builder for the four SPI modules.
    pub fn clock(self, handle: &mut ccm::Handle) -> WDOG<M> {
        let (ccm, _) = handle.raw();

        let addr: *const ral::wdog::RegisterBlock = &*self.reg;

        // enable the instance's clock
        match addr {
            wdog::WDOG1 => {
                ral::modify_reg!(ral::ccm, ccm, CCGR3, CG8: 0b11);
            }
            wdog::WDOG2 => {
                ral::modify_reg!(ral::ccm, ccm, CCGR5, CG5: 0b11);
            }
            _ => {}
        };

        WDOG::new(self.reg)
    }
}

pub struct WDOG<M> {
    reg: Instance,
    _module: PhantomData<M>,
}

impl<M> WDOG<M>
where
    M: Unsigned,
{
    fn new(reg: ral::wdog::Instance) -> Self {
        let mut wdog = WDOG {
            reg,
            _module: PhantomData,
        };
        wdog.begin();
        wdog
    }

    fn begin(&mut self) {
        ral::modify_reg!(ral::wdog, self.reg, WCR, SRS: SRS_1, WT: 0b1111 );
        ral::modify_reg!(ral::wdog, self.reg, WICR, WTIS: WTIS_1, WICT: 0b11 );
        ral::modify_reg!(
            ral::wdog,
            self.reg,
            WCR,
            WDE: WDE_1,
            WDA: WDA_1,
            WDT: WDT_1,
            SRE: SRE_1
        );
        ral::modify_reg!(ral::wdog, self.reg, WMCR, PDE: PDE_0);
    }

    pub fn feed(&mut self) {
        ral::write_reg!(ral::wdog, self.reg, WSR, 0x5555);
        ral::write_reg!(ral::wdog, self.reg, WSR, 0xAAAA);
    }
}

impl<M: Unsigned> embedded_hal::watchdog::Watchdog for WDOG<M> {
    fn feed(&mut self) {
        self.feed()
    }
}
