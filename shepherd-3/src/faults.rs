use strum::{EnumCount, VariantArray, EnumIter};

#[derive(EnumCount, VariantArray, EnumIter)]
pub enum FaultId {
    Fault1,
    Fault2,
    Fault3,
}
impl FaultId {
    pub const CONFIG: [u32; FaultId::COUNT] = [1, 2, 3];
} 