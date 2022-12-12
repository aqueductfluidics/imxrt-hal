#[cfg(test)]
mod tests;

use core::convert::TryFrom;
use core::ops::{Deref, DerefMut};

use super::id::{ExtendedId, Id, StandardId};

/// A CAN data or remote frame.
#[derive(Clone, Debug, Eq)]
pub struct Frame {
    pub(crate) code: CodeReg,
    pub(crate) id: IdReg,
    pub(crate) data: Data,
}

impl Frame {
    /// Creates a new data frame.
    pub fn new_data(id: impl Into<Id>, data: impl Into<Data>) -> Self {
        let (code, id) = match id.into() {
            Id::Standard(id) => (CodeReg::new_standard(), IdReg::new_standard(id)),
            Id::Extended(id) => (CodeReg::new_extended(), IdReg::new_extended(id)),
        };

        Self {
            code,
            id,
            data: data.into(),
        }
    }

    /// Creates a new data frame.
    #[inline(always)]
    pub fn new_from_raw(code: u32, id: u32, data: impl Into<Data>) -> Self {
        Self {
            code: CodeReg::new(code),
            id: IdReg::new(id),
            data: data.into(),
        }
    }

    /// Creates a new remote frame with configurable data length code (DLC).
    ///
    /// # Panics
    ///
    /// This function will panic if `dlc` is not inside the valid range `0..=8`.
    pub fn new_remote(id: impl Into<Id>, dlc: u8) -> Self {
        assert!(dlc <= 8);

        let mut frame = Self::new_data(id, []);
        // Just extend the data length, even with no data present. The API does not hand out this
        // `Data` object.
        frame.data.len = dlc;
        frame.code = frame.code.with_rtr(true);
        frame
    }

    /// Returns true if this frame is an extended frame.
    #[inline(always)]
    pub fn is_extended(&self) -> bool {
        self.code.is_extended()
    }

    /// Returns true if this frame is a standard frame.
    #[inline(always)]
    pub fn is_standard(&self) -> bool {
        self.code.is_standard()
    }

    /// Returns true if this frame is a remote frame.
    #[inline(always)]
    pub fn is_remote_frame(&self) -> bool {
        self.code.rtr()
    }

    /// Returns true if this frame is a data frame.
    #[inline(always)]
    pub fn is_data_frame(&self) -> bool {
        !self.is_remote_frame()
    }

    /// Returns the frame identifier.
    #[inline(always)]
    pub fn id(&self) -> Id {
        if self.is_extended() {
            self.id.to_extended()
        } else {
            self.id.to_standard()
        }
    }

    /// Returns the data length code (DLC) which is in the range 0..8.
    ///
    /// For data frames the DLC value always matches the length of the data.
    /// Remote frames do not carry any data, yet the DLC can be greater than 0.
    #[inline(always)]
    pub fn dlc(&self) -> u8 {
        self.data.len() as u8
    }

    #[inline(always)]
    pub fn to_tx_once_code(&self) -> u32 {
        ((self.dlc() as u32) << CodeReg::DLC_SHIFT) | FlexCanMailboxCSCode::TxOnce.to_code_reg()
    }

    /// Returns the frame data (0..8 bytes in length) if this is a data frame.
    ///
    /// If this is a remote frame, returns `None`.
    pub fn data(&self) -> Option<&Data> {
        if self.is_data_frame() {
            Some(&self.data)
        } else {
            None
        }
    }
}

impl PartialEq for Frame {
    fn eq(&self, other: &Self) -> bool {
        match (self.data(), other.data()) {
            (None, None) => self.id.eq(&other.id),
            (Some(a), Some(b)) => self.id.eq(&other.id) && a.eq(b),
            (None, Some(_)) | (Some(_), None) => false,
        }
    }
}

#[derive(Debug, PartialEq)]
#[repr(u8)]
pub enum FlexCanMailboxCSCode {
    RxInactive = 0b0000,
    RxEmpty = 0b0100,
    RxFull = 0b0010,
    RxOverrun = 0b0110,
    RxAnswer = 0b1010,
    RxBusy = 0b0001,

    TxInactive = 0b1000,
    TxAbort = 0b1001,
    TxOnce = 0b1100,
    TxAnswer = 0b1110,

    Unknown,
}

impl ::core::convert::TryFrom<u8> for FlexCanMailboxCSCode {
    type Error = ();

    #[inline(always)]
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            v if v == FlexCanMailboxCSCode::RxInactive as u8 => {
                Ok(FlexCanMailboxCSCode::RxInactive)
            }
            v if v == FlexCanMailboxCSCode::RxEmpty as u8 => Ok(FlexCanMailboxCSCode::RxEmpty),
            v if v == FlexCanMailboxCSCode::RxFull as u8 => Ok(FlexCanMailboxCSCode::RxFull),
            v if v == FlexCanMailboxCSCode::RxOverrun as u8 => Ok(FlexCanMailboxCSCode::RxOverrun),
            v if v == FlexCanMailboxCSCode::RxAnswer as u8 => Ok(FlexCanMailboxCSCode::RxAnswer),
            v if v == FlexCanMailboxCSCode::RxBusy as u8 => Ok(FlexCanMailboxCSCode::RxBusy),
            v if v == FlexCanMailboxCSCode::TxInactive as u8 => {
                Ok(FlexCanMailboxCSCode::TxInactive)
            }
            v if v == FlexCanMailboxCSCode::TxAbort as u8 => Ok(FlexCanMailboxCSCode::TxAbort),
            v if v == FlexCanMailboxCSCode::TxOnce as u8 => Ok(FlexCanMailboxCSCode::TxOnce),
            v if v == FlexCanMailboxCSCode::TxAnswer as u8 => Ok(FlexCanMailboxCSCode::TxAnswer),
            _ => Ok(FlexCanMailboxCSCode::Unknown),
        }
    }
}

impl FlexCanMailboxCSCode {
    #[inline(always)]
    pub fn to_code_reg(&self) -> u32 {
        (*self as u32) << CodeReg::CODE_SHIFT
    }

    #[inline(always)]
    pub fn from_code_reg(reg: u32) -> Result<Self, ()> {
        Self::try_from(((reg & CodeReg::CODE_MASK) >> CodeReg::CODE_SHIFT) as u8)
    }

    #[inline(always)]
    pub fn is_tx_mailbox(&self) -> bool {
        (*self as u8) >> 3 != 0
    }
}

/// Code Register of a FlexCAN mailbox.
///
/// Contains info relating to the frame's remote status (RTR), whether
/// the frams is standard os extended (IDE), the length of the data
/// content (DLC), and the timestamp.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CodeReg(u32);

impl CodeReg {
    pub(crate) const CODE_SHIFT: u32 = 24;
    pub(crate) const CODE_MASK: u32 = 0b1111_u32 << Self::CODE_SHIFT;

    #[allow(dead_code)]
    const SRR_SHIFT: u32 = 22;
    #[allow(dead_code)]
    const SRR_MASK: u32 = 0b1_u32 << Self::SRR_SHIFT;

    const IDE_SHIFT: u32 = 21;
    const IDE_MASK: u32 = 0b1_u32 << Self::IDE_SHIFT;

    const RTR_SHIFT: u32 = 20;
    const RTR_MASK: u32 = 0b1_u32 << Self::RTR_SHIFT;

    const DLC_SHIFT: u32 = 16;
    const DLC_MASK: u32 = 0b111_u32 << Self::DLC_SHIFT;

    const TIMESTAMP_SHIFT: u32 = 0;
    const TIMESTAMP_MASK: u32 = 0xFFFF_u32 << Self::TIMESTAMP_SHIFT;

    /// Creates a new code reg.
    pub fn new(code: u32) -> Self {
        Self(code)
    }

    /// Creates a new code reg for a standard identifier.
    pub fn new_standard() -> Self {
        Self::new(0x00)
    }

    /// Creates a new code reg for an extended identifier.
    pub fn new_extended() -> Self {
        Self::new(Self::IDE_MASK)
    }

    /// Returns the register value.
    #[inline(always)]
    pub fn to_code_reg(&self) -> u32 {
        self.0
    }

    /// Set the 4 bit code content for a CodeReg
    #[inline(always)]
    pub fn set_code(&mut self, code: FlexCanMailboxCSCode) -> Self {
        Self::new(self.0 & ((code as u32) << Self::CODE_SHIFT))
    }

    /// Get the 4 bit code content for a CodeReg.
    ///
    /// # Panics
    ///
    /// This function will panic if a matching [`FlexCanMailboxCSCode`] is not found.
    #[inline(always)]
    pub fn code(&self) -> FlexCanMailboxCSCode {
        FlexCanMailboxCSCode::try_from(((self.0 & Self::CODE_MASK) >> Self::CODE_SHIFT) as u8)
            .unwrap()
    }

    pub fn timestamp(&self) -> u16 {
        ((self.0 & Self::TIMESTAMP_MASK) >> Self::TIMESTAMP_SHIFT) as u16
    }

    pub fn dlc(&self) -> u8 {
        ((self.0 & Self::DLC_MASK) >> Self::DLC_SHIFT) as u8
    }

    /// Returns `true` if the code reg is an extended identifier.
    #[inline(always)]
    fn is_extended(self) -> bool {
        self.0 & Self::IDE_MASK != 0
    }

    /// Returns `true` if the code reg is a standard identifier.
    #[inline(always)]
    fn is_standard(self) -> bool {
        !self.is_extended()
    }

    /// Sets the remote transmission (RTR) flag. This marks the identifier as
    /// being part of a remote frame.
    #[must_use = "returns a new IdReg without modifying `self`"]
    pub fn with_rtr(self, rtr: bool) -> Self {
        if rtr {
            Self::new(self.0 | Self::RTR_MASK)
        } else {
            Self::new(self.0 & !Self::RTR_MASK)
        }
    }

    /// Returns `true` if the identifer is part of a remote frame (RTR bit set).
    #[inline(always)]
    pub fn rtr(self) -> bool {
        self.0 & Self::RTR_MASK != 0
    }
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
pub struct IdReg(u32);

impl IdReg {
    const STANDARD_SHIFT: u32 = 18;
    const EXTENDED_SHIFT: u32 = 0;

    /// Creates a new standard identifier (11bit, Range: 0..0x7FF)
    ///
    /// Panics for IDs outside the allowed range.
    #[inline(always)]
    pub fn new(id: u32) -> Self {
        Self(id)
    }

    /// Returns the register value.
    #[inline(always)]
    pub fn to_id_reg(&self) -> u32 {
        self.0
    }

    /// Creates a new standard identifier (11bit, Range: 0..0x7FF)
    ///
    /// Panics for IDs outside the allowed range.
    #[inline(always)]
    pub fn new_standard(id: StandardId) -> Self {
        let id = u32::from(id.as_raw()) << Self::STANDARD_SHIFT;
        Self::new(id)
    }

    /// Creates a new extendended identifier (29bit , Range: 0..0x1FFFFFFF).
    ///
    /// Panics for IDs outside the allowed range.
    #[inline(always)]
    pub fn new_extended(id: ExtendedId) -> Self {
        let id = u32::from(id.as_raw()) << Self::EXTENDED_SHIFT;
        Self::new(id)
    }

    #[inline(always)]
    pub fn to_standard(&self) -> Id {
        Id::Extended(unsafe { ExtendedId::new_unchecked(self.0 >> Self::STANDARD_SHIFT) })
    }

    #[inline(always)]
    pub fn to_extended(&self) -> Id {
        Id::Standard(unsafe { StandardId::new_unchecked((self.0 >> Self::EXTENDED_SHIFT) as u16) })
    }
}

/// Payload of a CAN data frame.
///
/// Contains 0 to 8 Bytes of data.
///
/// `Data` implements `From<[u8; N]>` for all `N` up to 8, which provides a convenient lossless
/// conversion from fixed-length arrays.
#[derive(Debug, Copy, Clone)]
pub struct Data {
    pub(crate) len: u8,
    pub(crate) bytes: [u8; 8],
}

impl Data {
    /// Creates a data payload from a raw byte slice.
    ///
    /// Returns `None` if `data` contains more than 8 Bytes (which is the maximum).
    ///
    /// `Data` can also be constructed from fixed-length arrays up to length 8 via `From`/`Into`.
    pub fn new(data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }

        let mut bytes = [0; 8];
        bytes[..data.len()].copy_from_slice(data);

        Some(Self {
            len: data.len() as u8,
            bytes,
        })
    }

    /// Creates an empty data payload containing 0 bytes.
    #[inline(always)]
    pub const fn empty() -> Self {
        Self {
            len: 0,
            bytes: [0; 8],
        }
    }
}

impl Deref for Data {
    type Target = [u8];

    #[inline(always)]
    fn deref(&self) -> &[u8] {
        &self.bytes[..usize::from(self.len)]
    }
}

impl DerefMut for Data {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut [u8] {
        &mut self.bytes[..usize::from(self.len)]
    }
}

impl AsRef<[u8]> for Data {
    #[inline(always)]
    fn as_ref(&self) -> &[u8] {
        self.deref()
    }
}

impl AsMut<[u8]> for Data {
    #[inline(always)]
    fn as_mut(&mut self) -> &mut [u8] {
        self.deref_mut()
    }
}

impl PartialEq for Data {
    fn eq(&self, other: &Self) -> bool {
        self.as_ref() == other.as_ref()
    }
}

impl Eq for Data {}

#[cfg(feature = "unstable-defmt")]
impl defmt::Format for Data {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        self.as_ref().format(fmt)
    }
}

macro_rules! data_from_array {
    ( $($len:literal),+ ) => {
        $(
            impl From<[u8; $len]> for Data {
                #[inline(always)]
                fn from(arr: [u8; $len]) -> Self {
                    let mut bytes = [0; 8];
                    bytes[..$len].copy_from_slice(&arr);
                    Self {
                        len: $len,
                        bytes,
                    }
                }
            }
        )+
    };
}

data_from_array!(0, 1, 2, 3, 4, 5, 6, 7, 8);
