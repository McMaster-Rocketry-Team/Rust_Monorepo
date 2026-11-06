use std::sync::Arc;
use std::sync::RwLock;
use std::time::Instant;
use std::time::SystemTime;
use std::time::UNIX_EPOCH;
use anyhow::Result;
use firmware_common_new::vlp::packets::VLPDownlinkPacket;
use firmware_common_new::vlp::packets::VLPUplinkPacket;
use lora_phy::mod_params::PacketStatus;
use std::time::Duration;

use axum::{
    Json, extract::Request, Router, extract::{WebSocketUpgrade, State, ws::{Message, WebSocket}}, http::{header, Method, StatusCode}, middleware::{self, Next}, response::{IntoResponse, Response}, routing::{any, post},
};

use serde::{Serialize};
// use serde_json::json;
use tokio::sync::{broadcast, Mutex};

use crate::gs::{config::GroundStationConfig, vlp_client::VLPClientTrait};

pub async fn start_server(
    client: &'static (dyn VLPClientTrait + Sync),
    config: Arc<RwLock<GroundStationConfig>>,
) -> Result<()> {

    let (tx, _rx) = broadcast::channel::<String>(100);

    let state = AppState {tx: tx.clone(), client: client, uplink_lock: Arc::new(Mutex::new(()))};

    // spawn data polling task from client
    tokio::spawn(async move {
        let mut last_packet_time = Instant::now();
        loop {
            if let Some((packet, status)) = client.try_receive() {
                // println!("{:#?}", packet);
                // println!("rssi {}, snr {}", status.rssi, status.snr);
                /* 
                TODO: sending timestamps instead of received ones
                Packet decoding and unit corrections can be seen in the formatting in gs/downlink_packet_display/sections (L259)

                on each received `VLPDownlinkPacket`, decode it and flatten it
                    into `(key, value, timestampMs)` triples using the `gsd_*` dictionary —
                    the flattening rules (booleans as 0/1, enums as their numeric values,
                    fixed-point fields as floats) are exactly what the catalog already
                    declares. Include the link-quality pseudo-signals (RSSI, SNR,
                    `gsd_packet_type`, packet counters) which come from the LoRa
                    `PacketStatus`, not the payload.
                 */
                let packet_time = Instant::now();
                let packet_millis = match SystemTime::now().duration_since(UNIX_EPOCH) {
                    Ok(t) => t.as_millis(),
                    Err(_) => {println!("System Time before UNIX EPOCH");0}
                };
                let time_since_last: u16 = match packet_time.duration_since(last_packet_time).as_secs().try_into() {
                    Ok(t) => t,
                    Err(_) => 120, // we return 120 if it does not fit, this is max range frontend expects
                };
                last_packet_time = packet_time;
                let dat = downlink_triples(&packet, &status, packet_millis, time_since_last);
                // safe to ignore error, just shows no clients connected
                for trip in dat {
                    let _ = tx.send(format!("({}, {}, {})", trip.0, trip.1, trip.2));
                    }
            } else {
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        }
    });

    let app = Router::new()
        .route("/ws", any(ws_handler))
        .route("/cmd", post(command_handler))
        .layer(middleware::from_fn(cors_middleware))
        .with_state(state);

    let listener = tokio::net::TcpListener::bind("0.0.0.0:3000").await.unwrap();
    axum::serve(listener, app).await.unwrap();
    Ok(())
}

// CORS
async fn cors_middleware(req: Request, next: Next) -> Response {
    let is_preflight = req.method() == Method::OPTIONS;

    // Return 204 for OPTIONS, or forward actual requests to handlers
    let mut response = if is_preflight {
        StatusCode::NO_CONTENT.into_response()
    } else {
        next.run(req).await
    };

    let headers = response.headers_mut();
    headers.insert(header::ACCESS_CONTROL_ALLOW_ORIGIN, "*".parse().unwrap());
    headers.insert(
        header::ACCESS_CONTROL_ALLOW_METHODS,
        "POST, GET, OPTIONS".parse().unwrap(),
    );
    headers.insert(
        header::ACCESS_CONTROL_ALLOW_HEADERS,
        "content-type, accept".parse().unwrap(),
    );

    response
}

#[derive(Clone)]
struct AppState {
    tx: broadcast::Sender<String>,
    client: &'static (dyn VLPClientTrait + Sync),
    uplink_lock: Arc<Mutex<()>>,
}

#[derive(Serialize)]
struct UplinkResponse {
    rssi: i16,
    snr: i16,
    message: String,
}

async fn command_handler(State(state): State<AppState>, Json(payload): Json<VLPUplinkPacket>) -> (StatusCode, Json<UplinkResponse>) {
    // let _lock = match state.uplink_lock.lock().await {
    //     Ok(guard) => guard,
    //     Err(_) => {
    //         return (
    //             StatusCode::CONFLICT,
    //             Json(UplinkResponse { rssi: 404, snr: 404, message: "Uplink already in progess".to_string() }),
    //         )
    //     }
    // };
    let _lock = state.uplink_lock.lock().await;
    match payload {
        VLPUplinkPacket::SetTargetApogee(ref apogee) => {
            println!("Setting target apogee to: {apogee:?}m");
        }
        VLPUplinkPacket::ChangeMode(ref mode) => {
            println!("Transitioning mode to: {mode:?}");
        }
        VLPUplinkPacket::Reset(ref device) => {
            println!("Resetting subsystem: {device:?}");
        }
        VLPUplinkPacket::AMPOutputOverwrite(ref amp_data) => {
            println!("Applying AMP overwrite: {amp_data:?}");
        }
        // VLPUplinkPacket::PayloadEPSOutputOverwrite(eps_data) => {
        //     println!("Applying EPS overwrite: {eps_data:?}");
        // }
        VLPUplinkPacket::FirePyro(ref pyro) => {
            println!("Firing pyro channel: {pyro:?}");
        }
    }
    // let packet: VLPUplinkPacket;
    state.client.send_nb(payload.clone());
    let timeout = Duration::from_secs(5);
    let start = Instant::now();

    let send_result = loop {
        if let Some(res) = state.client.try_get_send_result() {
            break res;
        }

        if start.elapsed() > timeout {
            return (
                StatusCode::GATEWAY_TIMEOUT,
                Json(UplinkResponse {
                    rssi: 404,
                    snr: 404,
                    message: "TIMEOUT".to_string(),
                }),
            );
        }

        tokio::time::sleep(Duration::from_millis(15)).await;
    };

    match send_result {
        Ok(status) => (
            StatusCode::OK,
            Json(UplinkResponse {
                rssi: status.rssi,
                snr: status.snr,
                message: "OK".to_string(),
            }),
        ),
        Err(err) => (
            StatusCode::BAD_GATEWAY,
            Json(UplinkResponse {
                rssi: 404,
                snr: 404,
                message: format!("Uplink transmission failed: {err:?}").to_string(),
            }),
        )
    }

}

async fn ws_handler(ws: WebSocketUpgrade, State(state): State<AppState>) -> Response {
    let rx = state.tx.subscribe();
    ws.on_upgrade(move |socket| handle_socket(socket, rx))
}

async fn handle_socket(mut socket: WebSocket, mut rx: broadcast::Receiver<String>) {
    while let Ok(data) = rx.recv().await {
        if socket.send(Message::Text(data.into())).await.is_err() {
            // Client disconnected or socket closed
            println!("Breaking");
            break;
        }
    }
}

fn downlink_triples(packet: &VLPDownlinkPacket, status: &PacketStatus, timestamp: u128, time_since_last: u16) -> Vec<(&'static str, f64, u128)> {
    let mut triples = Vec::new();
    // we always need rssi and snr and time since last packet
    triples.push(("gsd_rssi", status.rssi.into(), timestamp));
    triples.push(("gsd_snr", status.snr.into(), timestamp));
    triples.push(("gsd_seconds_since_received", time_since_last.into(), timestamp));
    match packet {
        VLPDownlinkPacket::Telemetry(p) => {
            triples.push(("gsd_packet_type", 1.0, timestamp));
            triples.push(("gsd_unix_clock_ready", p.unix_clock_ready().into(), timestamp));
            triples.push(("gsd_num_of_fix_satellites", p.num_of_fix_satellites().into(), timestamp));
            let (lat, lon) = match p.lat_lon() {
                Some(t) => t,
                None => (0.0,0.0) // TODO: what should defaults be in this case?
            };
            triples.push(("gsd_lat", lat, timestamp));
            triples.push(("gsd_lon", lon, timestamp));
            // lat_lon_valid: true,
            triples.push(("gsd_air_temperature", p.air_temperature().into(), timestamp));
            triples.push(("gsd_vl_battery_v", p.vl_battery_v().into(), timestamp));

            triples.push(("gsd_pyro_main_continuity", p.pyro_main_continuity().into(), timestamp));
            triples.push(("gsd_pyro_drogue_continuity", p.pyro_drogue_continuity().into(), timestamp));

            triples.push(("gsd_flight_stage", p.flight_stage() as i8 as f64, timestamp)); // TODO: the flightstage definition here does not match the one given in NICK documentation
            let alt_agl = match p.deployment_kf_altitude_agl() {
                Some(t) => t,
                None => 0.0 // TODO: is there really no velocity or altitude tracking at all during mach lockout? 
                // TODO: frontend doesn't have keys for altitude or velocity validity - should these be added?
            };
            let max_alt_agl = match p.max_deployment_kf_altitude_agl() {
                Some(t) => t,
                None => 0.0
            };
            let vert_velo = match p.deployment_kf_vertical_velocity() {
                Some(t) => t,
                None => 0.0
            };
            triples.push(("gsd_altitude_agl", alt_agl.into(), timestamp));
            triples.push(("gsd_max_altitude_agl", max_alt_agl.into(), timestamp));
            triples.push(("gsd_air_speed", vert_velo.into(), timestamp));
            // deployment_kf_valid: false,
            // deployment_kf_altitude_agl: 162,
            // deployment_kf_vertical_velocity: 282,

            // max_deployment_kf_altitude_valid: true,
            // max_deployment_kf_altitude_agl: 2595,

            // airbrakes_kf_tilt_valid: true,
            // airbrakes_kf_tilt_deg: 133,
            let tilt_deg = match p.airbrakes_kf_tilt_deg() {
                Some(t) => t,
                None => 0.0
            };
            triples.push(("gsd_tilt_deg", tilt_deg.into(), timestamp)); // TODO: again, frontend does not expect validity key

            // TODO: frontend expects none of these, should it?
            // airbrakes_enabled: true,
            // airbrakes_calibrated: true,
            // mpc_predicted_apogee_valid: true,
            // mpc_predicted_apogee_agl: 4704,
            // target_apogee_agl: 5028,
            
            triples.push(("gsd_shared_battery_v", p.shared_battery_v().unwrap_or_else(||0.0).into(), timestamp));
            triples.push(("gsd_amp_online", p.amp_online().into(), timestamp));
            triples.push(("gsd_amp_rebooted_in_last_5s", p.amp_rebooted_in_last_5s().into(), timestamp));

            triples.push(("gsd_amp_out1", p.amp_out1() as i8 as f64, timestamp));
            triples.push(("gsd_amp_out1_overwrote", p.amp_out1_overwrote().into(), timestamp));
            triples.push(("gsd_amp_out2", p.amp_out2() as i8 as f64, timestamp));
            triples.push(("gsd_amp_out2_overwrote", p.amp_out2_overwrote().into(), timestamp));
            triples.push(("gsd_amp_out3", p.amp_out3() as i8 as f64, timestamp));
            triples.push(("gsd_amp_out3_overwrote", p.amp_out3_overwrote().into(), timestamp));


            // icarus_status_valid: false,
            triples.push(("gsd_icarus_online", p.icarus_online().into(), timestamp));
            triples.push(("gsd_icarus_rebooted_in_last_5s", p.icarus_rebooted_in_last_5s().into(), timestamp));
            triples.push(("gsd_air_brakes_commanded_extension_percentage", p.air_brakes_commanded_extension_percentage().into(), timestamp));
            let air_brakes_act = match p.air_brakes_actual_extension_percentage() {
                Some(t) => t,
                None => 0.0 // TODO: correct default?
            };
            let air_brake_temp = match p.air_brakes_servo_temp() {
                Some(t) => t,
                None => 0.0
            };
            triples.push(("gsd_air_brakes_actual_extension_percentage", air_brakes_act.into(), timestamp));
            triples.push(("gsd_air_brakes_servo_temp", air_brake_temp.into(), timestamp));

            triples.push(("gsd_ozys1_online", p.ozys_online().into(), timestamp));
            triples.push(("gsd_ozys1_rebooted_in_last_5s", p.ozys_rebooted_in_last_5s().into(), timestamp));
            //TODO: frontend expects OZYS2 as well, not provided here

            
            // TODO: again what is EPM?
            // epm_batt_v: 61,
            // epm_sys_3v3_ma: 4,
            // epm_sys_5v_ma: 11,
            // epm_per_3v3_ma: 2,
            // epm_per_5v_ma: 24,
            // epm_per_9v_ma: 30,
            // epm_per_12v_ma: 31,

            
            // Frontend doesn't seem to want anything from here down
            // payload_sdrm_online: false,
            // payload_sdrm_rebooted_in_last_5s: false,
            // payload_epm_alive: true,
            // payload_sem_alive: true,
            // payload_epm_rails_on: true,
            // payload_exp1_active: false,
            // payload_exp2_active: true,
            // payload_exp3_active: false,
            // payload_sdrm_sd_logging: true,
            // payload_sem_sd_logging: false,
            // payload_arm_seq_running: false,
            // payload_arm_seq_complete: true,
            // payload_arm_seq_fault: false,

            // sem_actuator_1_steps: 0,
            // sem_actuator_2_steps: 19,
            // sem_actuator_3_steps: 540,
            // sem_load_cell_1_n: 54,
            // sem_load_cell_2_n: 131,
            // sem_load_cell_3_n: 255,

            // payload_exp1_fractured: true,
            // payload_exp1_finished: true,
            // payload_exp1_fault: false,
            // payload_exp1_closure_confirmed: true,
            // payload_exp1_monitoring: false,
            // payload_exp2_fractured: false,
            // payload_exp2_finished: false,
            // payload_exp2_fault: false,
            // payload_exp2_closure_confirmed: true,
            // payload_exp2_monitoring: true,
            // payload_exp3_fractured: false,
            // payload_exp3_finished: false,
            // payload_exp3_fault: true,
            // payload_exp3_closure_confirmed: false,
            // payload_exp3_monitoring: false,
        },
        VLPDownlinkPacket::LowPowerTelemetry(p) => {
            triples.push(("gsd_packet_type", 2.0, timestamp)); // PacketType enum from NICK docs has LowPowerTelemetry as 2
            triples.push(("gsd_num_of_fix_satellites", p.num_of_fix_satellites().into(), timestamp));
            let (lat, lon) = match p.lat_lon() {
                Some(t) => t,
                None => (0.0,0.0) // TODO: what should defaults be in this case?
            };
            triples.push(("gsd_lat", lat, timestamp));
            triples.push(("gsd_lon", lon, timestamp));
            triples.push(("gsd_air_temperature", p.air_temperature().into(), timestamp));
            triples.push(("gsd_vl_battery_v", p.vl_battery_v().into(), timestamp));
            triples.push(("gsd_shared_battery_v", p.shared_battery_v().unwrap_or_else(||0.0).into(), timestamp)); // again here, what is the default?
            triples.push(("gsd_amp_online", p.amp_online.into(), timestamp));
            // "gps_fixed": p.gps_fixed(), this was not specified in NICK expected keys, but is present here
            //TODO: what are these? are these an old version of EPS?
            // "epm_batt_v": p.epm_batt_v().volts(),
            // "epm_batt_below_range": p.epm_batt_v() == EpmBattV::BelowRange,
        },
        VLPDownlinkPacket::SelfTestResult(p) => {
            triples.push(("gsd_packet_type", 4.0, timestamp));
        },
        VLPDownlinkPacket::LandedTelemetry(p) => {
            triples.push(("gsd_packet_type", 3.0, timestamp));
            let (lat, lon) = match p.lat_lon() {
                Some(t) => t,
                None => (0.0,0.0) // TODO: what should defaults be in this case?
            };
            triples.push(("gsd_lat", lat, timestamp));
            triples.push(("gsd_lon", lon, timestamp));
            triples.push(("gsd_num_of_fix_satellites", p.num_of_fix_satellites().into(), timestamp));
            // lat_lon_valid: true,
            // battery_v: 801, Is this vl_battery_v?
            triples.push(("gsd_shared_battery_v", p.shared_battery_v().unwrap_or_else(||0.0).into(), timestamp));
            triples.push(("gsd_amp_online", p.amp_online().into(), timestamp));
            triples.push(("gsd_amp_rebooted_in_last_5s", p.amp_rebooted_in_last_5s().into(), timestamp));
            // This enum casting relies on front and back end agreeing on the order of the enum, then again, so would literals
            triples.push(("gsd_amp_out1", p.amp_out1() as i8 as f64, timestamp));
            triples.push(("gsd_amp_out1_overwrote", p.amp_out1_overwrote().into(), timestamp));
            triples.push(("gsd_amp_out2", p.amp_out2() as i8 as f64, timestamp));
            triples.push(("gsd_amp_out2_overwrote", p.amp_out2_overwrote().into(), timestamp));
            triples.push(("gsd_amp_out3", p.amp_out3() as i8 as f64, timestamp));
            triples.push(("gsd_amp_out3_overwrote", p.amp_out3_overwrote().into(), timestamp));
            // TODO: keys expects an amp4 but not provided here, is this a new addition?
        },
        VLPDownlinkPacket::Ack(_) => {

        },
        // json!({
        //     "type": "ack_downlink", "rssi": status.rssi, "snr": status.snr,
        // }),
    };
    triples
}