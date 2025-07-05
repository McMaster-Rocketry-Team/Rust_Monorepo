#![no_std]

mod fmt;
mod state_propagation;

#[cfg(test)]
mod tests;

pub fn add(left: f32, right: f32) -> f32 {
    left + right
}
