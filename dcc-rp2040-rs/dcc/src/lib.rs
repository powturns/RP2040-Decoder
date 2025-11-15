#![no_std]

use crate::cv::store::{Store, StoreExt};
use crate::transport::{Packet, PacketError};

mod device;
mod log;
pub mod transport;
pub mod cv;
pub mod handler;

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
pub fn is_recipient(packet: &Packet, store: &impl Store) -> bool {
    packet.addr().map(|a| a == store.addr()).unwrap_or(false)
}


pub trait Timer {
    /// Clears any running timers.
    fn stop(&mut self);

    /// Resets the timer
    fn start(&mut self);

    /// Calculates the time since the previous reset in ms.
    fn elapsed(&self) -> Option<usize>;
}