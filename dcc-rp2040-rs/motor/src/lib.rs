#![no_std]
extern crate alloc;

mod speed;

#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(test, derive(core::fmt::Debug))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Direction {
    Forward,
    Reverse,
}
