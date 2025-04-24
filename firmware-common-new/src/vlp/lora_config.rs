use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LoraConfig {
    pub frequency: u32,
    pub sf: u8,
    pub bw: u32,
    pub cr: u8,
    pub power: i32,
}

impl LoraConfig {
    pub fn sf_modulation(&self) -> lora_modulation::SpreadingFactor {
        use lora_modulation::SpreadingFactor;

        match self.sf {
            5 => SpreadingFactor::_5,
            6 => SpreadingFactor::_6,
            7 => SpreadingFactor::_7,
            8 => SpreadingFactor::_8,
            9 => SpreadingFactor::_9,
            10 => SpreadingFactor::_10,
            11 => SpreadingFactor::_11,
            12 => SpreadingFactor::_12,
            _ => panic!("Invalid spreading factor"),
        }
    }

    pub fn sf_phy(&self) -> lora_phy::mod_params::SpreadingFactor {
        use lora_phy::mod_params::SpreadingFactor;

        match self.sf {
            5 => SpreadingFactor::_5,
            6 => SpreadingFactor::_6,
            7 => SpreadingFactor::_7,
            8 => SpreadingFactor::_8,
            9 => SpreadingFactor::_9,
            10 => SpreadingFactor::_10,
            11 => SpreadingFactor::_11,
            12 => SpreadingFactor::_12,
            _ => panic!("Invalid spreading factor"),
        }
    }

    pub fn bw_modulation(&self) -> lora_modulation::Bandwidth {
        use lora_modulation::Bandwidth;

        match self.bw {
            7810u32 => Bandwidth::_7KHz,
            10420u32 => Bandwidth::_10KHz,
            15630u32 => Bandwidth::_15KHz,
            20830u32 => Bandwidth::_20KHz,
            31250u32 => Bandwidth::_31KHz,
            41670u32 => Bandwidth::_41KHz,
            62500u32 => Bandwidth::_62KHz,
            125000u32 => Bandwidth::_125KHz,
            250000u32 => Bandwidth::_250KHz,
            500000u32 => Bandwidth::_500KHz,
            _ => panic!("Invalid bandwidth"),
        }
    }

    pub fn bw_phy(&self) -> lora_phy::mod_params::Bandwidth {
        use lora_phy::mod_params::Bandwidth;
        match self.bw {
            7810u32 => Bandwidth::_7KHz,
            10420u32 => Bandwidth::_10KHz,
            15630u32 => Bandwidth::_15KHz,
            20830u32 => Bandwidth::_20KHz,
            31250u32 => Bandwidth::_31KHz,
            41670u32 => Bandwidth::_41KHz,
            62500u32 => Bandwidth::_62KHz,
            125000u32 => Bandwidth::_125KHz,
            250000u32 => Bandwidth::_250KHz,
            500000u32 => Bandwidth::_500KHz,
            _ => panic!("Invalid bandwidth"),
        }
    }

    pub fn cr_modulation(&self) -> lora_modulation::CodingRate {
        use lora_modulation::CodingRate;
        match self.cr {
            5 => CodingRate::_4_5,
            6 => CodingRate::_4_6,
            7 => CodingRate::_4_7,
            8 => CodingRate::_4_8,
            _ => panic!("Invalid coding rate"),
        }
    }

    pub fn cr_phy(&self) -> lora_phy::mod_params::CodingRate {
        use lora_phy::mod_params::CodingRate;
        match self.cr {
            5 => CodingRate::_4_5,
            6 => CodingRate::_4_6,
            7 => CodingRate::_4_7,
            8 => CodingRate::_4_8,
            _ => panic!("Invalid coding rate"),
        }
    }
}

impl Into<lora_modulation::BaseBandModulationParams> for &LoraConfig {
    fn into(self) -> lora_modulation::BaseBandModulationParams {
        lora_modulation::BaseBandModulationParams::new(
            self.sf_modulation(),
            self.bw_modulation(),
            self.cr_modulation(),
        )
    }
}
