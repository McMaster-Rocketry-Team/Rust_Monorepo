use super::{
    packets::{ack::AckPacket, VLPDownlinkPacket, VLPUplinkPacket, MAX_VLP_PACKET_SIZE},
    radio::Radio,
};
use embassy_futures::yield_now;
use embassy_sync::{blocking_mutex::raw::RawMutex, signal::Signal};
use lora_phy::mod_params::{PacketStatus, RadioError};
use sha2::Digest;
use sha2::Sha256;

/// buffer contains data without ecc and free space for ecc
/// data_len is the length of data in the buffer
/// returns the length of data with ecc in the buffer
/// 
/// | data len | ecc len | total len |
/// | -------- | ------- | --------- |
/// | 1        | 2       | 3         |
/// | 2        | 2       | 4         |
/// | 3        | 2       | 5         |
/// | 4        | 2       | 6         |
/// | 5        | 2       | 7         |
/// | 6        | 2       | 8         |
/// | 7        | 2       | 9         |
/// | 8        | 2       | 10        |
/// | 9        | 2       | 11        |
/// | 10       | 2       | 12        |
/// | 11       | 2       | 13        |
/// | 12       | 3       | 15        |
/// | n        | n // 4  | n + n // 4|
fn encode_ecc(buffer: &mut [u8], data_len: usize) -> usize {
    let ecc_len = if data_len < 12 {
        2
    } else {
        data_len / 4
    };
    let encoder = reed_solomon::Encoder::new(ecc_len);
    let encoded = encoder.encode(&buffer[..data_len]);
    buffer[data_len..(data_len + ecc_len)].copy_from_slice(&encoded.ecc());
    data_len + ecc_len
}

/// buffer contains data with ecc
/// returns the length of data in the buffer if ecc is correct
/// returns None if ecc is incorrect
fn decode_ecc(buffer: &mut [u8]) -> Option<usize> {
    let ecc_len = if buffer.len() < 15 {
        2
    } else {
        buffer.len() / 5
    };
    let decoder = reed_solomon::Decoder::new(ecc_len);
    if let Ok(recovered) = decoder.correct(buffer, None) {
        let recovered_data = recovered.data();
        buffer[..recovered_data.len()].copy_from_slice(recovered_data);
        Some(recovered_data.len())
    } else {
        None
    }
}

pub struct VLPGroundStation<M: RawMutex> {
    tx_signal: Signal<M, VLPUplinkPacket>,
    tx_result_signal: Signal<M, Result<PacketStatus, VLPTXError>>,
    rx_signal: Signal<M, (VLPDownlinkPacket, PacketStatus)>,
}

impl<M: RawMutex> VLPGroundStation<M> {
    pub fn new() -> Self {
        VLPGroundStation {
            tx_signal: Signal::new(),
            rx_signal: Signal::new(),
            tx_result_signal: Signal::new(),
        }
    }

    /// Calling send multiple times concurrently is not supported
    /// Returns the packet status of the ack message
    /// Calling send while receiving a packet is supported
    pub async fn send(&self, packet: VLPUplinkPacket) -> Result<PacketStatus, VLPTXError> {
        self.tx_signal.signal(packet);
        self.tx_result_signal.wait().await
    }

    pub async fn receive(&self) -> (VLPDownlinkPacket, PacketStatus) {
        self.rx_signal.wait().await
    }

    pub fn daemon<'a, 'b, 'c>(
        &'a self,
        radio: &'b mut impl Radio,
        key: &'c [u8; 32],
    ) -> VLPGroundStationDaemon<'a, 'b, 'c, M, impl Radio> {
        VLPGroundStationDaemon::new(self, radio, key)
    }
}

pub struct VLPGroundStationDaemon<'a, 'b, 'c, M: RawMutex, R: Radio> {
    client: &'a VLPGroundStation<M>,
    buffer: [u8; MAX_VLP_PACKET_SIZE],
    radio: &'b mut R,
    key: &'c [u8; 32],
}

impl<'a, 'b, 'c, M: RawMutex, R: Radio> VLPGroundStationDaemon<'a, 'b, 'c, M, R> {
    pub fn new(client: &'a VLPGroundStation<M>, radio: &'b mut R, key: &'c [u8; 32]) -> Self {
        VLPGroundStationDaemon {
            client,
            buffer: [0u8; MAX_VLP_PACKET_SIZE],
            radio,
            key,
        }
    }

    pub async fn run(&mut self) {
        loop {
            if let Err(e) = self.rx_tx_cycle().await {
                log_warn!("Error in VLP rx_tx_cycle: {:?}", e);
                yield_now().await;
            }
        }
    }

    async fn rx_tx_cycle(&mut self) -> Result<(), VLPDaemonError> {
        let (rx_len, packet_status) = self
            .radio
            .rx(&mut self.buffer, None)
            .await
            .map_err(VLPDaemonError::Radio)?;

        // decode ecc
        let rx_len = decode_ecc(&mut self.buffer[..rx_len]).ok_or(VLPDaemonError::ECCError)?;

        // deserialize the packet
        let rx_packet = VLPDownlinkPacket::deserialize(&self.buffer[..rx_len])
            .ok_or(VLPDaemonError::DeserializeError)?;
        self.client.rx_signal.signal((rx_packet, packet_status));

        if let Some(tx_packet) = self.client.tx_signal.try_take() {
            self.client
                .tx_result_signal
                .signal(self.tx(rx_len, tx_packet).await);
        }

        Ok(())
    }

    async fn tx(
        &mut self,
        rx_len: usize,
        tx_packet: VLPUplinkPacket,
    ) -> Result<PacketStatus, VLPTXError> {
        let mut hasher = Sha256::new();
        hasher.update(self.key); // hash 1: shared key
        hasher.update(&self.buffer[..rx_len]); // hash 2: downlink packet without ecc

        // packet
        let mut offset = tx_packet.serialize(&mut self.buffer);

        // sign with sha256
        hasher.update(&mut self.buffer[..offset]); // hash 3: uplink packet without ecc
        let hash = hasher.finalize();
        self.buffer[offset..(offset + 16)].copy_from_slice(&hash[0..16]);
        offset += 16;
        let expected_ack_sha = u16::from_be_bytes((&hash[0..2]).try_into().unwrap());

        // encode ecc
        offset = encode_ecc(&mut self.buffer, offset);

        // send the packet
        self.radio
            .tx(&self.buffer[..offset])
            .await
            .map_err(VLPTXError::Radio)?;

        match self.radio.rx(&mut self.buffer, Some(300)).await {
            Ok((rx_len, packet_status)) => {
                // decode ecc
                let rx_len =
                    decode_ecc(&mut self.buffer[..rx_len]).ok_or(VLPTXError::InvalidAck)?;

                if let Some(VLPDownlinkPacket::Ack(ack_packet)) =
                    VLPDownlinkPacket::deserialize(&self.buffer[..rx_len])
                    && ack_packet.sha == expected_ack_sha
                {
                    return Ok(packet_status);
                } else {
                    return Err(VLPTXError::InvalidAck);
                }
            }
            Err(RadioError::ReceiveTimeout) => {
                return Err(VLPTXError::AckNotReceived);
            }
            Err(e) => {
                return Err(VLPTXError::Radio(e));
            }
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
enum VLPDaemonError {
    Radio(RadioError),
    DeserializeError,
    SignatureError,
    ECCError,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum VLPTXError {
    Radio(RadioError),
    AckNotReceived,
    InvalidAck,
}

pub struct VLPAvionics<M: RawMutex> {
    tx_signal: Signal<M, VLPDownlinkPacket>,
    rx_signal: Signal<M, (VLPUplinkPacket, PacketStatus)>,
}

impl<M: RawMutex> VLPAvionics<M> {
    pub fn new() -> Self {
        VLPAvionics {
            tx_signal: Signal::new(),
            rx_signal: Signal::new(),
        }
    }

    pub fn send(&self, packet: VLPDownlinkPacket) {
        self.tx_signal.signal(packet);
    }

    pub async fn receive(&self) -> (VLPUplinkPacket, PacketStatus) {
        self.rx_signal.wait().await
    }

    pub fn daemon<'a, 'b, 'c>(
        &'a self,
        radio: &'b mut impl Radio,
        key: &'c [u8; 32],
    ) -> VLPAvionicsDaemon<'a, 'b, 'c, M, impl Radio> {
        VLPAvionicsDaemon::new(self, radio, key)
    }
}

pub struct VLPAvionicsDaemon<'a, 'b, 'c, M: RawMutex, R: Radio> {
    client: &'a VLPAvionics<M>,
    buffer: [u8; MAX_VLP_PACKET_SIZE],
    radio: &'b mut R,
    key: &'c [u8; 32],
}

impl<'a, 'b, 'c, M: RawMutex, R: Radio> VLPAvionicsDaemon<'a, 'b, 'c, M, R> {
    pub fn new(client: &'a VLPAvionics<M>, radio: &'b mut R, key: &'c [u8; 32]) -> Self {
        VLPAvionicsDaemon {
            client,
            buffer: [0u8; MAX_VLP_PACKET_SIZE],
            radio,
            key,
        }
    }

    pub async fn run(&mut self) {
        loop {
            if let Err(e) = self.cycle().await {
                log_warn!("Error in VLP cycle: {:?}", e);
                yield_now().await;
            }
        }
    }

    async fn cycle(&mut self) -> Result<(), VLPDaemonError> {
        let tx_packet = self.client.tx_signal.wait().await;

        // serialize the packet
        let mut offset = tx_packet.serialize(&mut self.buffer);

        let mut hasher = Sha256::new();
        hasher.update(self.key); // hash 1: shared key
        hasher.update(&self.buffer[..offset]); // hash 2: downlink packet without ecc

        // encode ecc
        offset = encode_ecc(&mut self.buffer, offset);

        self.radio
            .tx(&self.buffer[..offset])
            .await
            .map_err(VLPDaemonError::Radio)?;

        match self.radio.rx(&mut self.buffer, Some(300)).await {
            Ok((rx_len, packet_status)) => {
                return self
                    .process_rx_and_send_ack(rx_len, packet_status, hasher)
                    .await;
            }
            Err(RadioError::ReceiveTimeout) => {
                return Ok(());
            }
            Err(e) => {
                return Err(VLPDaemonError::Radio(e));
            }
        }
    }

    async fn process_rx_and_send_ack(
        &mut self,
        rx_len: usize,
        packet_status: PacketStatus,
        mut hasher: impl Digest,
    ) -> Result<(), VLPDaemonError> {
        // decode ecc
        let rx_len = decode_ecc(&mut self.buffer[..rx_len]).ok_or(VLPDaemonError::ECCError)?;

        if rx_len < 16 {
            return Err(VLPDaemonError::DeserializeError);
        }

        // verify the signature
        let packet = &self.buffer[..rx_len - 16];
        let received_signature = &self.buffer[rx_len - 16..rx_len];

        hasher.update(packet);
        let expected_signature = hasher.finalize();
        if received_signature != &expected_signature[0..16] {
            return Err(VLPDaemonError::SignatureError);
        }

        // deserialize the packet
        let packet =
            VLPUplinkPacket::deserialize(packet).ok_or(VLPDaemonError::DeserializeError)?;
        self.client.rx_signal.signal((packet, packet_status));

        // send ack
        let sha = u16::from_be_bytes((&expected_signature[0..2]).try_into().unwrap());
        self.send_ack(sha).await?;

        Ok(())
    }

    async fn send_ack(&mut self, sha: u16) -> Result<(), VLPDaemonError> {
        // construct the ack packet
        let ack_packet = VLPDownlinkPacket::Ack(AckPacket { sha });

        // serialize the packet
        let mut offset = ack_packet.serialize(&mut self.buffer);

        // encode ecc
        offset = encode_ecc(&mut self.buffer, offset);

        // send the packet
        self.radio
            .tx(&self.buffer[..offset])
            .await
            .map_err(VLPDaemonError::Radio)?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::assert_matches::assert_matches;

    use embassy_futures::{
        join::join,
        select::{select3, Either3},
    };
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;

    use crate::vlp::packets::{
        change_mode::{ChangeModePacket, Mode},
        low_power_telemetry::LowPowerTelemetryPacket,
    };

    use super::*;

    #[test]
    fn test_ecc() {
        // first 8 are data, last 1 is ecc
        let mut buffer: [u8; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 0, 0];
        let len = 8;

        // ecc encode
        let len = encode_ecc(&mut buffer, len);
        assert_eq!(len, 10);

        // should be able to decode if data is unchanged
        {
            let mut after_buffer = buffer.clone();
            let len = decode_ecc(&mut after_buffer).unwrap();
            assert_eq!(len, 8);
            assert_eq!(&after_buffer[..len], &buffer[..len]);
        }

        // should be able to decode if any 1 byte is changed
        for i in 0..10 {
            let mut after_buffer = buffer.clone();
            after_buffer[i] ^= 0xFF;
            let len = decode_ecc(&mut after_buffer).unwrap();
            assert_eq!(len, 8);
            assert_eq!(&after_buffer[..len], &buffer[..len]);
        }
    }

    struct MockRadioPair<M: RawMutex> {
        a_to_b_data: Signal<M, Vec<u8>>,
        b_to_a_data: Signal<M, Vec<u8>>,
    }

    impl<M: RawMutex> MockRadioPair<M> {
        fn new() -> Self {
            MockRadioPair {
                a_to_b_data: Signal::new(),
                b_to_a_data: Signal::new(),
            }
        }

        fn radio_a(&self) -> RadioA<M> {
            RadioA { pair: self }
        }

        fn radio_b(&self) -> RadioB<M> {
            RadioB { pair: self }
        }
    }

    struct RadioA<'a, M: RawMutex> {
        pair: &'a MockRadioPair<M>,
    }

    impl<'a, M: RawMutex> Radio for RadioA<'a, M> {
        async fn tx(&mut self, buffer: &[u8]) -> Result<(), RadioError> {
            let mut data = buffer.to_vec();
            data[0] = 0xFF; // simulate a corruption in the first byte
            self.pair.a_to_b_data.signal(data);
            Ok(())
        }

        async fn rx(
            &mut self,
            buffer: &mut [u8],
            _timeout_ms: Option<u16>,
        ) -> Result<(usize, PacketStatus), RadioError> {
            let data = self.pair.b_to_a_data.wait().await;
            let len = data.len();
            buffer[..len].copy_from_slice(&data);
            Ok((len, PacketStatus { rssi: 0, snr: 0 }))
        }
    }

    struct RadioB<'a, M: RawMutex> {
        pair: &'a MockRadioPair<M>,
    }

    impl<'a, M: RawMutex> Radio for RadioB<'a, M> {
        async fn tx(&mut self, buffer: &[u8]) -> Result<(), RadioError> {
            let mut data = buffer.to_vec();
            data[0] = 0xFF; // simulate a corruption in the first byte
            self.pair.b_to_a_data.signal(data);
            Ok(())
        }

        async fn rx(
            &mut self,
            buffer: &mut [u8],
            _timeout_ms: Option<u16>,
        ) -> Result<(usize, PacketStatus), RadioError> {
            let data = self.pair.a_to_b_data.wait().await;
            let len = data.len();
            buffer[..len].copy_from_slice(&data);
            Ok((len, PacketStatus { rssi: 0, snr: 0 }))
        }
    }

    #[tokio::test]
    async fn test_vlp_client_downlink() {
        let ground_station_client = VLPGroundStation::<NoopRawMutex>::new();
        let avionics_client = VLPAvionics::<NoopRawMutex>::new();

        let radio_pair = MockRadioPair::<NoopRawMutex>::new();
        let mut radio_a = radio_pair.radio_a();
        let mut radio_b = radio_pair.radio_b();
        let key = [0x69u8; 32];

        let mut ground_station_daemon = ground_station_client.daemon(&mut radio_a, &key);
        let ground_station_daemon_fut = ground_station_daemon.run();

        let mut avionics_daemon = avionics_client.daemon(&mut radio_b, &key);
        let avionics_daemon_fut = avionics_daemon.run();

        let ground_station_fut = async {
            let (packet, _) = ground_station_client.receive().await;
            assert_matches!(
                packet,
                VLPDownlinkPacket::LowPowerTelemetry(packet) if packet.num_of_fix_satellites() == 5
            );
        };
        let avionics_fut = async {
            avionics_client.send(VLPDownlinkPacket::LowPowerTelemetry(
                LowPowerTelemetryPacket::new(5, true, true, 8.2, 27.0),
            ));
        };

        assert_matches!(
            select3(
                ground_station_daemon_fut,
                avionics_daemon_fut,
                join(ground_station_fut, avionics_fut)
            )
            .await,
            Either3::Third(_)
        )
    }

    #[tokio::test]
    async fn test_vlp_client_uplink() {
        let ground_station_client = VLPGroundStation::<NoopRawMutex>::new();
        let avionics_client = VLPAvionics::<NoopRawMutex>::new();

        let radio_pair = MockRadioPair::<NoopRawMutex>::new();
        let mut radio_a = radio_pair.radio_a();
        let mut radio_b = radio_pair.radio_b();
        let key = [0x69u8; 32];

        let mut ground_station_daemon = ground_station_client.daemon(&mut radio_a, &key);
        let ground_station_daemon_fut = ground_station_daemon.run();

        let mut avionics_daemon = avionics_client.daemon(&mut radio_b, &key);
        let avionics_daemon_fut = avionics_daemon.run();

        let ground_station_fut = async {
            let send_fut = async {
                let send_result = ground_station_client
                    .send(VLPUplinkPacket::ChangeMode(ChangeModePacket {
                        mode: Mode::ReadyToLaunch,
                    }))
                    .await;

                assert!(send_result.is_ok());
            };

            let receive_fut = async {
                let (packet, _) = ground_station_client.receive().await;
                assert_matches!(
                    packet,
                    VLPDownlinkPacket::LowPowerTelemetry(packet) if packet.num_of_fix_satellites() == 5
                );
            };

            join(send_fut, receive_fut).await;
        };
        let avionics_fut = async {
            avionics_client.send(VLPDownlinkPacket::LowPowerTelemetry(
                LowPowerTelemetryPacket::new(5, true, true, 8.2, 27.0),
            ));

            let (received_packet, _) = avionics_client.receive().await;
            assert_matches!(
                received_packet,
                VLPUplinkPacket::ChangeMode(ChangeModePacket {
                    mode: Mode::ReadyToLaunch
                })
            );
        };

        assert_matches!(
            select3(
                ground_station_daemon_fut,
                avionics_daemon_fut,
                join(ground_station_fut, avionics_fut)
            )
            .await,
            Either3::Third(_)
        )
    }
}
