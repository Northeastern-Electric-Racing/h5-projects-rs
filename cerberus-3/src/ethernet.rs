use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{Peri, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use nereth::{NerEth, NerPublisher};
use variant_count::VariantCount;

const ID: u8 = 4;

pub struct EthernetResources {
    pub phy_reset: Peri<'static, peripherals::PE10>,
    pub rng: Peri<'static, peripherals::RNG>,
    pub eth: Peri<'static, peripherals::ETH>,
    pub ref_clk: Peri<'static, peripherals::PA1>,
    pub crs: Peri<'static, peripherals::PA7>,
    pub rx_d0: Peri<'static, peripherals::PC4>,
    pub rx_d1: Peri<'static, peripherals::PC5>,
    pub tx_d0: Peri<'static, peripherals::PB12>,
    pub tx_d1: Peri<'static, peripherals::PB15>,
    pub tx_en: Peri<'static, peripherals::PA5>,
    pub sma: Peri<'static, peripherals::ETH_SMA>,
    pub mdio: Peri<'static, peripherals::PA2>,
    pub mdc: Peri<'static, peripherals::PC1>,
}

/// Defines every Ethernet topic in one place.
/// Generates the `Topic` enum and a matching (key, unit) table, in the same order.
macro_rules! topics {
    ($($name:ident => $key:literal, $unit:literal;)*) => {
        #[derive(Clone, Copy, VariantCount)]
        pub enum Topic {
            $($name,)*
        }

        impl Topic {
            const TABLE: &'static [(&'static str, &'static str)] = &[$(($key, $unit),)*];
        }
    };
}

topics! {
    Acceleration => "VCU_Ethernet/A/Acceleration", "mg";
    Gyro         => "VCU_Ethernet/A/Gyro",         "mdps";
}

pub struct Packet {
    pub topic: Topic,
    pub values: [f32; 3],
}

pub static OUTGOING: Channel<ThreadModeRawMutex, Packet, 8> = Channel::new();

pub fn try_publish(topic: Topic, values: [f32; 3]) -> bool {
    OUTGOING.try_send(Packet { topic, values }).is_ok()
}

#[embassy_executor::task]
pub async fn ethernet_task(spawner: Spawner, r: EthernetResources) {
    let mut phy_reset = Output::new(r.phy_reset, Level::Low, Speed::Low);
    phy_reset.set_low();
    Timer::after_millis(500).await;
    phy_reset.set_high();

    let eth = NerEth::<ID>::new(
        spawner, r.rng, r.eth, r.ref_clk, r.crs, r.rx_d0, r.rx_d1, r.tx_d0, r.tx_d1, r.tx_en,
        r.sma, r.mdio, r.mdc,
    )
    .await
    .expect("Failed to initialize NerEth.");

    let mut publishers: heapless::Vec<NerPublisher<'static>, { Topic::VARIANT_COUNT }> =
        heapless::Vec::new();

    for &(key, unit) in Topic::TABLE {
        let Some(p) = eth.get_publisher(key, Some(unit)).await else {
            defmt::panic!("Failed to create publisher for {}", key);
        };
        let _ = publishers.push(p);
    }

    loop {
        let packet = OUTGOING.receive().await;
        publishers[packet.topic as usize]
            .send_values(packet.values)
            .await;
    }
}
