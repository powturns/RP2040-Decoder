#![no_std]
extern crate alloc;

use crate::cv::store::{Store, StoreExt};
use crate::transport::packet::Packet;
use bitflags::bitflags;

pub(crate) mod log;

pub mod cv;
mod device;
pub mod handler;
pub mod transport;

#[cfg(test)]
mod testing;

const BROADCAST_ADDRESS: u16 = 0xFFFF;

/// Reads the extended address from the start of the given data.
///
/// Data must contain at least two elements, with the most significant address bytes
/// stored in the first element.
fn read_extended_address(data: &[u8]) -> u16 {
    assert!(data.len() >= 2);

    ((data[0] & 0b00111111) as u16) << 8 | (data[1] as u16)
}

/// Checks to see if the packet is addressed to the specific decoder.
pub(crate) fn is_recipient(packet: &Packet, store: &impl Store) -> bool {
    packet.addr().map(|a|
                          store.addr().map(|b| a == b).unwrap_or(false)
    ).unwrap_or(false)
}

/// Checks to see if the packet is addressed to the broadcast address.
pub(crate) fn is_broadcast(packet: &Packet) -> bool {
    packet.addr().map(|a| a == BROADCAST_ADDRESS).unwrap_or(false)
}

pub trait Timer {
    /// Clears any running timers.
    fn stop(&mut self);

    /// Resets the timer
    fn start(&mut self);

    /// Calculates the time since the previous reset in ms.
    fn elapsed(&self) -> Option<usize>;
}

#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(test, derive(Debug))]
pub struct FunctionGroup {
    /// Flags that are present in the current group
    group_mask: FunctionGroupFlags,

    /// Value of the flags
    flags: FunctionGroupFlags,
}

impl FunctionGroup {

    /// Calculates the union between the other flags and these flags, taking into account the group mask.
    pub fn union_with_mask(&self, other: FunctionGroupFlags) -> FunctionGroupFlags {
        other.difference(self.group_mask).union(self.flags)
    }
}

impl FunctionGroup {
    fn new(group_mask: FunctionGroupFlags, flags: FunctionGroupFlags) -> Self {
        Self {
            group_mask,
            flags: flags.intersection(group_mask),
        }
    }
}

bitflags! {
    /// Represents the state of function groups.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct FunctionGroupFlags: u16 {
        const F1 = 1;
        const F2 = 1 <<1;
        const F3 = 1 <<2;
        const F4 = 1 <<3;
        const FL = 1 <<4;
        const F5 = 1 <<5;
        const F6 = 1 <<6;
        const F7 = 1 <<7;
        const F8 = 1 <<8;
        const F9 = 1 <<9;
        const F10 = 1 <<10;
        const F11 = 1 <<11;
        const F12 = 1 <<12;

        /// Functions in group 1
        const FG_1 = Self::F1.bits() | Self::F2.bits() | Self::F3.bits() | Self::F4.bits() | Self::FL.bits();

        /// Functions in group 2 (where s is 0)
        const FG_2_0 = Self::F9.bits() | Self::F10.bits() | Self::F11.bits() | Self::F12.bits();

        /// Functions in group 2 (where s is 1)
        const FG_2_1 = Self::F5.bits() | Self::F6.bits() | Self::F7.bits() | Self::F8.bits();
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for FunctionGroupFlags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "FunctionGroups({:013b})", self.bits())
    }
}
