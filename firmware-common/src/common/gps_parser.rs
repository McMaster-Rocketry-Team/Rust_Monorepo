use nmea::Nmea;

use crate::driver::gps::{NmeaSentence, GPS};

pub struct GPSParser<G: GPS> {
    gps: G,
    pub nmea: Nmea,
}

impl<G: GPS> GPSParser<G> {
    pub fn new(gps: G) -> Self {
        Self {
            gps,
            nmea: Nmea::default(),
        }
    }
    async fn reset(&mut self) {
        self.nmea = Nmea::default();
        self.gps.reset().await;
    }

    async fn set_enable_gps(&mut self, enable: bool) {
        self.gps.set_enable(enable).await;
    }

    pub fn update_one(&mut self) -> Option<(NmeaSentence, bool)> {
        while let Some(sentence) = self.gps.read_next_nmea_sentence() {
            let success = self.nmea.parse(&sentence.sentence.as_str()).is_ok();
            return Some((sentence, success));
        }
        return None;
    }
}