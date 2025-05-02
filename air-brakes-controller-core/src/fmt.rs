#![macro_use]
#![allow(unused_macros)]

#[cfg(feature = "defmt")]
#[derive(defmt::Format, Debug)]
#[allow(unused)]
pub struct Debug2DefmtWrapper<T: core::fmt::Debug>(#[defmt(Debug2Format)] pub T);

#[cfg(not(feature = "defmt"))]
#[derive(Debug)]
#[allow(unused)]
pub struct Debug2DefmtWrapper<T: core::fmt::Debug>(pub T);

macro_rules! log_trace {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "log")]
            ::log::trace!($s $(, $x)*);

            #[cfg(all(feature = "defmt", not(feature = "log")))]
            ::defmt::trace!($s $(, $x)*);
        }
    };
}

macro_rules! log_debug {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "log")]
            ::log::debug!($s $(, $x)*);

            #[cfg(all(feature = "defmt", not(feature = "log")))]
            ::defmt::debug!($s $(, $x)*);
        }
    };
}

macro_rules! log_info {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "log")]
            ::log::info!($s $(, $x)*);

            #[cfg(all(feature = "defmt", not(feature = "log")))]
            ::defmt::info!($s $(, $x)*);
        }
    };
}

macro_rules! log_warn {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "log")]
            ::log::warn!($s $(, $x)*);

            #[cfg(all(feature = "defmt", not(feature = "log")))]
            ::defmt::warn!($s $(, $x)*);
        }
    };
}

macro_rules! log_error {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "log")]
            ::log::error!($s $(, $x)*);

            #[cfg(all(feature = "defmt", not(feature = "log")))]
            ::defmt::error!($s $(, $x)*);
        }
    };
}

macro_rules! log_panic {
    ($s:literal $(, $x:expr)* $(,)?) => {
        #[allow(unreachable_code)]
        {
            #[cfg(feature = "log")]
            ::core::panic!($s $(, $x)*);

            #[cfg(all(feature = "defmt", not(feature = "log")))]
            ::defmt::panic!($s $(, $x)*);

            ::core::panic!();
        }
    };
}

macro_rules! log_unreachable {
    () => {
        #[allow(unreachable_code)]
        {
            #[cfg(feature = "log")]
            ::core::panic!("unreachable");

            #[cfg(all(feature = "defmt", not(feature = "log")))]
            ::defmt::unreachable!();

            ::core::panic!("unreachable");
        }
    };
}

macro_rules! log_assert {
    ($x:expr) => {{
        #[cfg(feature = "log")]
        assert!($x);

        #[cfg(feature = "defmt")]
        ::defmt::assert!($x);
    }};
}
