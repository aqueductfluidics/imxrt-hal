//! Filter bank API.

use super::{ExtendedId, Id, StandardId};

const F32_RTR: u32 = 0b010; // set the RTR bit to match remote frames
const F32_IDE: u32 = 0b100; // set the IDE bit to match extended identifiers
const F16_RTR: u16 = 0b10000;
const F16_IDE: u16 = 0b01000;

/// A 16-bit filter list entry.
///
/// This can match data and remote frames using standard IDs.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub struct ListEntry16(u16);

/// A 32-bit filter list entry.
///
/// This can match data and remote frames using extended or standard IDs.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub struct ListEntry32(u32);

/// A 16-bit identifier mask.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub struct Mask16 {
    id: u16,
    mask: u16,
}

/// A 32-bit identifier mask.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub struct Mask32 {
    id: u32,
    mask: u32,
}

impl ListEntry16 {
    /// Creates a filter list entry that accepts data frames with the given standard ID.
    ///
    /// This entry will *not* accept remote frames with the same ID.
    pub fn data_frames_with_id(id: StandardId) -> Self {
        Self(id.as_raw() << 5)
    }

    /// Creates a filter list entry that accepts remote frames with the given standard ID.
    pub fn remote_frames_with_id(id: StandardId) -> Self {
        Self(id.as_raw() << 5 | F16_RTR)
    }
}

impl ListEntry32 {
    /// Creates a filter list entry that accepts data frames with the given ID.
    ///
    /// This entry will *not* accept remote frames with the same ID.
    ///
    /// The filter will only accept *either* standard *or* extended frames, depending on `id`.
    pub fn data_frames_with_id(id: impl Into<Id>) -> Self {
        match id.into() {
            Id::Standard(id) => Self(u32::from(id.as_raw()) << 21),
            Id::Extended(id) => Self(id.as_raw() << 3 | F32_IDE),
        }
    }

    /// Creates a filter list entry that accepts remote frames with the given ID.
    pub fn remote_frames_with_id(id: impl Into<Id>) -> Self {
        match id.into() {
            Id::Standard(id) => Self(u32::from(id.as_raw()) << 21 | F32_RTR),
            Id::Extended(id) => Self(id.as_raw() << 3 | F32_IDE | F32_RTR),
        }
    }
}

impl Mask16 {
    /// Creates a 16-bit identifier mask that accepts all frames.
    ///
    /// This will accept both standard and extended data and remote frames with any ID.
    pub fn accept_all() -> Self {
        Self { id: 0, mask: 0 }
    }

    /// Creates a 16-bit identifier mask that accepts all frames with the given standard
    /// ID and mask combination.
    ///
    /// Filter logic: `frame_accepted = (incoming_id & mask) == (id & mask)`
    ///
    /// A mask of all all ones (`0x7FF`) matches an exact ID, a mask of 0 matches all IDs.
    ///
    /// Both data and remote frames with `id` will be accepted. Any extended frames will be
    /// rejected.
    pub fn frames_with_std_id(id: StandardId, mask: StandardId) -> Self {
        Self {
            id: id.as_raw() << 5,
            mask: mask.as_raw() << 5 | F16_IDE, // also require IDE = 0
        }
    }

    /// Make the filter accept data frames only.
    pub fn data_frames_only(&mut self) -> &mut Self {
        self.id &= !F16_RTR; // RTR = 0
        self.mask |= F16_RTR;
        self
    }

    /// Make the filter accept remote frames only.
    pub fn remote_frames_only(&mut self) -> &mut Self {
        self.id |= F16_RTR; // RTR = 1
        self.mask |= F16_RTR;
        self
    }
}

impl Mask32 {
    /// Creates a 32-bit identifier mask that accepts all frames.
    ///
    /// This will accept both standard and extended data and remote frames with any ID.
    pub fn accept_all() -> Self {
        Self { id: 0, mask: 0 }
    }

    /// Creates a 32-bit identifier mask that accepts all frames with the given extended
    /// ID and mask combination.
    ///
    /// Filter logic: `frame_accepted = (incoming_id & mask) == (id & mask)`
    ///
    /// A mask of all all ones (`0x1FFF_FFFF`) matches an exact ID, a mask of 0 matches all IDs.
    ///
    /// Both data and remote frames with `id` will be accepted. Standard frames will be rejected.
    pub fn frames_with_ext_id(id: ExtendedId, mask: ExtendedId) -> Self {
        Self {
            id: id.as_raw() << 3 | F32_IDE,
            mask: mask.as_raw() << 3 | F32_IDE, // also require IDE = 1
        }
    }

    /// Creates a 32-bit identifier mask that accepts all frames with the given standard
    /// ID and mask combination.
    ///
    /// Filter logic: `frame_accepted = (incoming_id & mask) == (id & mask)`
    ///
    /// A mask of all all ones (`0x7FF`) matches the exact ID, a mask of 0 matches all IDs.
    ///
    /// Both data and remote frames with `id` will be accepted. Extended frames will be rejected.
    pub fn frames_with_std_id(id: StandardId, mask: StandardId) -> Self {
        Self {
            id: u32::from(id.as_raw()) << 21,
            mask: u32::from(mask.as_raw()) << 21 | F32_IDE, // also require IDE = 0
        }
    }

    /// Make the filter accept data frames only.
    pub fn data_frames_only(&mut self) -> &mut Self {
        self.id &= !F32_RTR; // RTR = 0
        self.mask |= F32_RTR;
        self
    }

    /// Make the filter accept remote frames only.
    pub fn remote_frames_only(&mut self) -> &mut Self {
        self.id |= F32_RTR; // RTR = 1
        self.mask |= F32_RTR;
        self
    }
}

/// The configuration of a filter bank.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub enum BankConfig {
    List16([ListEntry16; 4]),
    List32([ListEntry32; 2]),
    Mask16([Mask16; 2]),
    Mask32(Mask32),
}

impl From<[ListEntry16; 4]> for BankConfig {
    #[inline]
    fn from(entries: [ListEntry16; 4]) -> Self {
        Self::List16(entries)
    }
}

impl From<[ListEntry32; 2]> for BankConfig {
    #[inline]
    fn from(entries: [ListEntry32; 2]) -> Self {
        Self::List32(entries)
    }
}

impl From<[Mask16; 2]> for BankConfig {
    #[inline]
    fn from(entries: [Mask16; 2]) -> Self {
        Self::Mask16(entries)
    }
}

impl From<Mask32> for BankConfig {
    #[inline]
    fn from(filter: Mask32) -> Self {
        Self::Mask32(filter)
    }
}

#[cfg(test)]
mod tests {
    // use super::*;
}
