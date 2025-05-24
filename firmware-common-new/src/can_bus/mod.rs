use core::future::Future;

pub mod id;
pub mod messages;
pub mod node_types;
pub mod receiver;
pub mod sender;
pub mod log_mutiplexer;

pub trait CanBusFrame {
    fn timestamp_us(&self) -> u64;
    fn id(&self) -> u32;
    fn data(&self) -> &[u8];
}

impl CanBusFrame for (u64, u32, &[u8]) {
    fn timestamp_us(&self) -> u64 {
        self.0
    }

    fn id(&self) -> u32 {
        self.1
    }

    fn data(&self) -> &[u8] {
        self.2
    }
}

impl<const N: usize> CanBusFrame for (u64, u32, &[u8; N]) {
    fn timestamp_us(&self) -> u64 {
        self.0
    }

    fn id(&self) -> u32 {
        self.1
    }

    fn data(&self) -> &[u8] {
        self.2
    }
}

pub trait CanBusTX {
    #[cfg(feature = "defmt")]
    type Error: defmt::Format + core::fmt::Debug;
    #[cfg(not(feature = "defmt"))]
    type Error: core::fmt::Debug;

    /// Send a message with the given ID and data. data must be
    /// not empty and not more than 8 bytes.
    fn send(&mut self, id: u32, data: &[u8]) -> impl Future<Output = Result<(), Self::Error>>;
}

pub trait CanBusRX {
    #[cfg(feature = "defmt")]
    type Error: defmt::Format + core::fmt::Debug;
    #[cfg(not(feature = "defmt"))]
    type Error: core::fmt::Debug;
    type Frame: CanBusFrame;

    fn receive(&mut self) -> impl Future<Output = Result<Self::Frame, Self::Error>>;
}
