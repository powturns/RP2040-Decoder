#![macro_use]
#![allow(unused)]

#[allow(unused_macros)]
#[collapse_debuginfo(yes)]
macro_rules! trace {
    ($s:literal $(, $x:expr_2021)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::trace!($s $(, $x)*);
            #[cfg(not(feature="defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[allow(unused_macros)]
#[collapse_debuginfo(yes)]
macro_rules! debug {
    ($s:literal $(, $x:expr_2021)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::debug!($s $(, $x)*);
            #[cfg(not(feature="defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[allow(unused_macros)]
#[collapse_debuginfo(yes)]
macro_rules! info {
    ($s:literal $(, $x:expr_2021)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::info!($s $(, $x)*);
            #[cfg(not(feature="defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[allow(unused_macros)]
#[collapse_debuginfo(yes)]
macro_rules! warn {
    ($s:literal $(, $x:expr_2021)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::warn!($s $(, $x)*);
            #[cfg(not(feature="defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[allow(unused_macros)]
#[collapse_debuginfo(yes)]
macro_rules! error {
    ($s:literal $(, $x:expr_2021)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::error!($s $(, $x)*);
            #[cfg(not(feature="defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}
