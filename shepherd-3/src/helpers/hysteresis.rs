#[derive(Clone, Copy)]
pub struct Hysteresis<T: PartialOrd> {
    active: bool,

    /// Threshold above which the state becomes active.
    upper_threshold: T,
    /// Threshold at or below which the state becomes inactive.
    lower_threshold: T,
}
impl<T: PartialOrd> Hysteresis<T> {
    /// Creates a new `Hysteresis` with `upper_threshold` and `lower_threshold`.
    pub const fn new(upper_threshold: T, lower_threshold: T) -> Self { 
        Self { active: false, upper_threshold, lower_threshold }
    }

    /// Creates a new `Hysteresis` with `upper_threshold` and `lower_threshold`.
    /// 
    /// If `lower_threshold >= upper_threshold`, this will return `Err(())`.
    pub fn new_checked(upper_threshold: T, lower_threshold: T) -> Result<Self, ()> { 
        if lower_threshold >= upper_threshold { return Err(()); }
        Ok(Self { active: false, upper_threshold, lower_threshold })
    }

    /// Updates the Hysteresis with `value`, and then returns the current state.
    /// 
    /// If `value` is above `upper_threshold, this will return `true`.
    /// 
    /// If `value` is between `upper_threshold` and `lower_threshold`:
    /// - This will return `true` if the state was already `true` (meaning we were above `upper_threshold` at some point, and even though we no longer are, we need to
    /// fall below `lower_threshold` to deassert the state).
    /// - This will return `false` if the state was already `false` (we must cross `upper_threshold` before the state asserts itself).
    /// 
    /// If `value` is at or below `lower_threshold`, this will return `false`.
    pub fn update(&mut self, value: T) -> bool {
        if value > self.upper_threshold {
            self.active = true;
            true
        } else if value <= self.lower_threshold {
            self.active = false;
            false
        } else {
            // `value` is between `upper_threshold` and `lower_threshold`
            if self.active { true } else { false }
        }
    }

    /// If this Hysteresis is currently active.
    /// 
    /// Note: This state could be stale depending on the last time you called `update()`. So you should probably consider that when calling this.
    pub fn active(&self) -> bool { self.active }
}