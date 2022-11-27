use super::CAN;

use crate::ccm;
use crate::iomuxc::consts::{Unsigned, U1, U2};
use crate::ral;

use core::cmp::Ord;
use core::convert::Infallible;
use core::marker::PhantomData;


/// A CAN ISO-TP builder that can build a CAN peripheral
pub struct IsoTPBuilder<M> {
    can: CAN<M>,
}

impl<M> IsoTPBuilder<M>
where
    M: Unsigned,
{
    pub fn new(can: CAN<M>) -> Self {
        IsoTPBuilder {
            can,
        }
    }

    pub fn build(self) -> IsoTP<M> {
        IsoTP::new(self.can)
    }
}

pub struct IsoTP<M> {
    pub can: CAN<M>,
}

impl<M> IsoTP<M> {
    pub fn new(can: CAN<M>) -> Self {
        Self { can }
    }
}