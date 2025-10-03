#![no_std]
#![no_main]

use defmt::info;
use embassy_time::Timer;
use mavlink::ardupilotmega::RC_CHANNELS_DATA;

const FIRE_COUNTER: u8 = 3;
const BURN_TIME: u64 = 1000; // ms

#[derive(Default)]
pub struct FireDetector {
    counter: u8,
    fired: bool,
}

impl FireDetector {
    pub fn tick(&mut self, rc: RC_CHANNELS_DATA) -> bool {
        match self.fired {
            true => {
                if rc.chan12_raw < 1500 {
                    self.fired = false;
                    self.counter = 0;
                }
                false
            }
            false => {
                if rc.chan12_raw >= 1500 && rc.chan9_raw >= 1200 && rc.chan9_raw < 1600 {
                    self.counter += 1;
                } else {
                    self.counter = 0;
                }

                if self.counter >= FIRE_COUNTER {
                    self.counter = 0;
                    self.fired = true;
                    true
                } else {
                    false
                }
            }
        }
    }
}

#[derive(Default)]
pub struct FireControl<const C: usize> {
    current: usize,
}

impl<const C: usize> FireControl<C> {
    pub async fn fire(&mut self) {
        info!("Fire {} start", self.current);
        Timer::after_millis(BURN_TIME).await;
        info!("Fire {} stop", self.current);

        self.current = if self.current >= C - 1 {
            0
        } else {
            self.current + 1
        }
    }
}
