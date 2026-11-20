//! One electrical model for the whole robot (PLAN.md D6): every motor's battery current through
//! the battery's and wiring's resistance, the bus voltage fed back to every motor, the main breaker
//! as a thermal model, brownout.
use model::Electrical;

#[derive(Debug, Clone)]
pub struct Bus {
    pub cfg: Electrical,
    /// volts at the controllers now
    pub voltage: f64,
    /// battery current, A (this step)
    pub current: f64,
    pub peak_current: f64,
    pub min_voltage: f64,
    /// energy out of the battery, J
    pub energy: f64,
    /// the main breaker's heat: trips at 1
    pub breaker_heat: f64,
    pub tripped: bool,
    pub brownout: bool,
    pub brownout_time: f64,
}

impl Bus {
    pub fn new(cfg: Electrical) -> Self {
        let v = cfg.battery_voltage;
        Bus { cfg, voltage: v, current: 0.0, peak_current: 0.0, min_voltage: v, energy: 0.0, breaker_heat: 0.0, tripped: false, brownout: false, brownout_time: 0.0 }
    }
    /// Takes this step's motor battery currents (A, summed) and works out the bus for the next.
    pub fn step(&mut self, motor_supply: f64, dt: f64) {
        let c = &self.cfg;
        let i = if self.tripped { 0.0 } else { motor_supply + c.fixed_load };
        let r = c.battery_resistance + c.wiring_resistance;
        let target = (c.battery_voltage - r * i).max(0.0);
        // the controllers see the sag after a millisecond or two (bulk capacitance)
        let k = (dt / 0.002).min(1.0);
        self.voltage += k * (target - self.voltage);
        self.current = i;
        self.peak_current = self.peak_current.max(i);
        self.min_voltage = self.min_voltage.min(self.voltage);
        self.energy += self.voltage * i * dt;
        // a thermal breaker: carries its rating forever, trips at about 2x in tens of seconds, 3x in a few
        let x = i / c.main_breaker;
        self.breaker_heat = (self.breaker_heat + dt * (x * x - 1.0) / 40.0).max(0.0);
        if self.breaker_heat >= 1.0 && c.trip_breakers { self.tripped = true; }
        self.brownout = self.voltage < c.brownout;
        if self.brownout { self.brownout_time += dt; }
    }
    pub fn reset_stats(&mut self) {
        self.peak_current = 0.0; self.min_voltage = self.voltage; self.energy = 0.0; self.brownout_time = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sags_and_trips() {
        let mut b = Bus::new(Electrical { trip_breakers: true, ..Electrical::default() });
        for _ in 0..1000 { b.step(194.0, 0.001); }
        // 200 A through 20 mOhm: 4 V down
        assert!((b.voltage - (12.8 - 0.02 * 200.0)).abs() < 1e-3, "{}", b.voltage);
        assert!(!b.tripped);
        for _ in 0..60_000 { b.step(400.0, 0.001); }
        assert!(b.tripped, "heat {}", b.breaker_heat);
    }
}
