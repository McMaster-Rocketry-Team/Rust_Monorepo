use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::signal::Signal;

pub struct SignalWithAck<M: RawMutex, T, A> {
    signal: Signal<M, T>,
    ack_signal: Signal<M, A>,
}

impl<M: RawMutex, T, A> SignalWithAck<M, T, A> {
    pub fn new() -> Self {
        Self {
            signal: Signal::new(),
            ack_signal: Signal::new(),
        }
    }

    pub fn split(
        &self,
    ) -> (
        SignalWithAckSender<'_, M, T, A>,
        SignalWithAckReceiver<'_, M, T, A>,
    ) {
        (
            SignalWithAckSender { parent: self },
            SignalWithAckReceiver { parent: self },
        )
    }
}

pub struct SignalWithAckSender<'a, M: RawMutex, T, A> {
    parent: &'a SignalWithAck<M, T, A>,
}

impl<'a, M: RawMutex, T, A> SignalWithAckSender<'a, M, T, A> {
    pub async fn send_and_wait_for_ack(&self, value: T) -> A {
        self.parent.signal.signal(value);
        self.parent.ack_signal.wait().await
    }
}

pub struct SignalWithAckReceiver<'a, M: RawMutex, T, A> {
    parent: &'a SignalWithAck<M, T, A>,
}

impl<'a, M: RawMutex, T, A> SignalWithAckReceiver<'a, M, T, A> {
    pub async fn wait(&self) -> T {
        self.parent.signal.wait().await
    }

    pub fn ack(&self, value: A) {
        self.parent.ack_signal.signal(value);
    }
}
