#![allow(warnings, unused)]

use embedded_io_async::ReadExactError;

use crate::rpc::half_duplex_serial::HalfDuplexSerial;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum RpcClientError<S: HalfDuplexSerial> {
    ECCMismatch,
    UnexpectedEof,
    Serial(S::Error),
}

impl<S: HalfDuplexSerial> From<ReadExactError<S::Error>> for RpcClientError<S> {
    fn from(e: ReadExactError<S::Error>) -> Self {
        match e {
            ReadExactError::UnexpectedEof => RpcClientError::UnexpectedEof,
            ReadExactError::Other(e) => RpcClientError::Serial(e),
        }
    }
}

#[macro_export]
macro_rules! create_rpc {
    {
        $rpc_name:ident
        $(enums {
            $(
                enum $enum_name:ident {
                    $( $enum_body:tt )*
                }
            )*
        })?
        $($rpc_i:literal $name:ident | $($req_var_name:ident: $req_var_type:ty),* | -> ($($res_var_name:ident: $res_var_type:ty),*))*
    } => {
        paste::paste! {
            // define enums
            $(
                $(
                    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                    #[derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize, Debug, Clone, PartialEq)]
                    pub enum $enum_name {
                        $( $enum_body )*
                    }
                )*
            )?

            $(
                // define request structs
                #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                #[derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize, Debug, Clone)]
                pub struct [< $name:camel Request >] {
                    $(
                        pub $req_var_name: $req_var_type,
                    )*
                }

                // define response structs
                #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                #[derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize, Debug, Clone)]
                pub struct [< $name:camel Response >] {
                    $(
                        pub $res_var_name: $res_var_type,
                    )*
                }
            )*

            pub trait [< $rpc_name:camel RpcServer >] {
                $(
                    fn [< $name:snake >](&mut self, $($req_var_name: $req_var_type, )*) -> impl core::future::Future<Output=[< $name:camel Response >]>;
                )*

                fn run_server<S: $crate::rpc::half_duplex_serial::HalfDuplexSerial>(&mut self, serial: &mut S) -> impl core::future::Future<Output=Result<(), S::Error>> {
                    use core::mem::size_of;
                    use crc::{Crc, CRC_8_SMBUS};
                    use embedded_io_async::ReadExactError;
                    use aligned::{Aligned, A2};
                    use rkyv::{
                        Archive,
                        api::low::{from_bytes_unchecked, to_bytes_in_with_alloc},
                        rancor::{Infallible, Failure},
                        ser::{allocator::SubAllocator, writer::Buffer}
                    };

                    async {
                        let crc = Crc::<u8>::new(&CRC_8_SMBUS);
                        let mut alloc = [MaybeUninit::<u8>::uninit(); 0];

                        const REQUEST_STRUCT_MAX_SIZE: usize = $crate::max_const!(
                            $(
                                size_of::<< [< $name:camel Request >] as Archive>::Archived>(),
                            )*
                        );
                        let mut request_buffer: Aligned<A2, [u8; $crate::max_const!(REQUEST_STRUCT_MAX_SIZE, 2)]> = Aligned([0u8; $crate::max_const!(REQUEST_STRUCT_MAX_SIZE, 2)]);
                        const RESPONSE_STRUCT_MAX_SIZE: usize = $crate::max_const!(
                            $(
                                size_of::<< [< $name:camel Request >] as Archive>::Archived>(),
                            )*
                        );
                        let mut response_buffer: Aligned<A2, [u8; {RESPONSE_STRUCT_MAX_SIZE + 16}]> = Aligned([0u8; {RESPONSE_STRUCT_MAX_SIZE + 16}]);

                        loop {
                            match serial.read_exact(&mut request_buffer[..2]).await {
                                Ok(_) => {},
                                Err(ReadExactError::UnexpectedEof)=>{
                                    log_warn!("Unexpected EOF, skipping.");
                                    continue;
                                },
                                Err(ReadExactError::Other(e))=>{
                                    Err(e)?;
                                }
                            }
                            let received_crc = request_buffer[0];
                            
                            let mut digest = crc.digest();
                            digest.update(&[request_buffer[1]]);

                            match request_buffer[1] {
                                $(
                                    $rpc_i => {
                                        let request_size = size_of::<<[< $name:camel Request >] as Archive>::Archived>();
                                        let response_size = size_of::<<[< $name:camel Response >] as Archive>::Archived>();

                                        match serial.read_exact(&mut request_buffer[..request_size]).await {
                                            Ok(_) => {},
                                            Err(ReadExactError::UnexpectedEof)=>{
                                                log_warn!("Unexpected EOF, skipping.");
                                                continue;
                                            },
                                            Err(ReadExactError::Other(e))=>{
                                                Err(e)?;
                                            }
                                        }

                                        digest.update(&request_buffer[..request_size]);
                                        let calculated_crc = digest.finalize();
                                        if calculated_crc != received_crc {
                                            log_warn!("Command CRC mismatch, skipping. received: {}, calculated: {}", received_crc, calculated_crc);
                                            continue;
                                        }

                                        #[allow(unused_variables)]
                                        let request = unsafe {
                                            from_bytes_unchecked::<[< $name:camel Request >], Infallible>(&request_buffer[..request_size]).unwrap()
                                        };
                                        let response = self.[< $name:snake >]($(request.$req_var_name, )*).await;

                                        to_bytes_in_with_alloc::<_, _, Failure>(
                                            &response,
                                            Buffer::from(&mut (*response_buffer)[..response_size]),
                                            SubAllocator::new(&mut alloc),
                                        )
                                        .unwrap();

                                        response_buffer[response_size] = crc.checksum(&response_buffer[..response_size]);
                                        serial.write_all(&response_buffer[..(response_size + 1)]).await?;
                                        log_info!("Response sent, crc: {}", response_buffer[response_size]);
                                    }
                                )*
                                255 if received_crc == 0x42 => {
                                    serial.write_all(&[255, 0x69]).await?;
                                }
                                id => {
                                    log_warn!("Unknown rpc id: {}", id);
                                }
                            }
                        }
                    }
                }
            }

            pub struct [< $rpc_name:camel RpcClient >]<'a, S: $crate::rpc::half_duplex_serial::HalfDuplexSerial> {
                serial: &'a mut S,
                crc: crc::Crc::<u8>,
                request_buffer: aligned::Aligned<aligned::A2, [u8; $crate::max_const!(
                    $(
                        size_of::<< [< $name:camel Request >] as rkyv::Archive>::Archived>(),
                    )*
                ) + 16]>,
                response_buffer: aligned::Aligned<aligned::A2, [u8; $crate::max_const!(
                    $(
                        size_of::<< [< $name:camel Response >] as rkyv::Archive>::Archived>(),
                    )*
                ) + 1]>
            }

            impl<'a, S: $crate::rpc::half_duplex_serial::HalfDuplexSerial> [< $rpc_name:camel RpcClient >]<'a, S> {
                pub fn new(serial: &'a mut S) -> Self {
                    Self {
                        serial,
                        crc: crc::Crc::<u8>::new(&crc::CRC_8_SMBUS),
                        request_buffer: aligned::Aligned([0u8; $crate::max_const!(
                            $(
                                size_of::<< [< $name:camel Request >] as rkyv::Archive>::Archived>(),
                            )*
                        ) + 16]),
                        response_buffer: aligned::Aligned([0u8; $crate::max_const!(
                            $(
                                size_of::<< [< $name:camel Response >] as rkyv::Archive>::Archived>(),
                            )*
                        ) + 1]),
                    }
                }

                pub async fn reset(&mut self) -> Result<bool, $crate::rpc::create_rpc::RpcClientError<S>> {
                    use $crate::rpc::create_rpc::RpcClientError;
                    use core::mem::size_of;

                    const REQUEST_STRUCT_MAX_SIZE: usize = $crate::max_const!(
                        $(
                            size_of::<< [< $name:camel Request >] as rkyv::Archive>::Archived>(),
                        )*
                    );

                    // flush the serial buffer
                    self.serial.write_all(&[255; REQUEST_STRUCT_MAX_SIZE]).await.map_err(RpcClientError::Serial)?;
                    self.serial.clear_read_buffer().await.map_err(RpcClientError::Serial)?;

                    // send reset command
                    self.serial.write_all(&[255, 0x42]).await.map_err(RpcClientError::Serial)?;

                    let mut buffer = [0u8; 2];
                    self.serial.read_exact(&mut buffer).await?;
                    if buffer == [255, 0x69] {
                        Ok(true)
                    } else {
                        Ok(false)
                    }
                }

                $(
                    pub async fn [< $name:snake >](&mut self, $($req_var_name: $req_var_type, )*) -> Result<[< $name:camel Response >], $crate::rpc::create_rpc::RpcClientError<S>> {
                        use core::mem::size_of;
                        use $crate::rpc::create_rpc::RpcClientError;
                        use rkyv::{
                            Archive,
                            api::low::{from_bytes_unchecked, to_bytes_in_with_alloc},
                            rancor::{Infallible, Failure},
                            ser::{allocator::SubAllocator, writer::Buffer}
                        };

                        let request_size = size_of::<<[< $name:camel Request >] as Archive>::Archived>();
                        let response_size = size_of::<<[< $name:camel Response >] as Archive>::Archived>();

                        let request = [< $name:camel Request >] {
                            $(
                                $req_var_name,
                            )*
                        };

                        to_bytes_in_with_alloc::<_, _, Failure>(
                            &request,
                            Buffer::from(&mut self.request_buffer[16..(request_size + 16)]),
                            SubAllocator::new(&mut [MaybeUninit::<u8>::uninit(); 0]),
                        )
                        .unwrap();

                        self.request_buffer[15] = $rpc_i;
                        self.request_buffer[14] = self.crc.checksum(&self.request_buffer[15..(request_size + 16)]);
                        
                        self.serial.write_all(&self.request_buffer[14..(request_size + 16)]).await.map_err(RpcClientError::Serial)?;
                        self.serial.read_exact(&mut self.response_buffer[..(response_size + 1)]).await?;

                        let received_crc = self.response_buffer[response_size];
                        let calculated_crc = self.crc.checksum(&self.response_buffer[..response_size]);

                        if calculated_crc != received_crc {
                            log_warn!("Response CRC mismatch, received: {}, calculated: {}",received_crc, calculated_crc);
                            return Err(RpcClientError::ECCMismatch);
                        }

                        unsafe {
                            Ok(from_bytes_unchecked::<[< $name:camel Response >], Infallible>(&self.response_buffer[..response_size]).unwrap())
                        }
                    }
                )*
            }

        }
    }
}
