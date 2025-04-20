use core::future::Future;

mod id;
pub mod messages;
pub mod node_types;
pub mod receiver;
pub mod sender;

pub trait CanBusRawMessage {
    fn timestamp(&self) -> f64;
    fn id(&self) -> u32;
    fn rtr(&self) -> bool;
    fn data(&self) -> &[u8];
}

pub trait CanBusTX {
    #[cfg(feature = "defmt")]
    type Error: defmt::Format + core::fmt::Debug;
    #[cfg(not(feature = "defmt"))]
    type Error: core::fmt::Debug;

    fn send(&mut self, id: u32, data: &[u8; 8]) -> impl Future<Output = Result<(), Self::Error>>;
}

pub trait CanBusRX {
    #[cfg(feature = "defmt")]
    type Error: defmt::Format + core::fmt::Debug;
    #[cfg(not(feature = "defmt"))]
    type Error: core::fmt::Debug;
    type Message: CanBusRawMessage;

    fn receive(&mut self) -> impl Future<Output = Result<Self::Message, Self::Error>>;
}
