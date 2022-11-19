//! Driver for the STM32 Touch Sensing Controller.
//!
//! This controller is the same across several STM32 families, and this driver
//! endeavors to support them all -- though it may not have a complete list now
//! it can readily be extended.
//!
//! When you create a Tsc object, it accesses the TSC registers directly. This
//! goes somewhat against the pattern of singleton peripherals, obtained from
//! one `take()` call, but it seems to be necessary. If the crate accepts a TSC
//! RegisterBlock from the user, we will run into versioning problems because there
//! doesn't appear to be any way to ensure that the `touch` crate uses the same
//! version of the PAC crate as the top-level or HAL crate. This doesn't really
//! present any problem, unless of course you alter TSC registers elsewhere in your
//! application, in which case things may not work as expected.
//!

#[cfg(feature="stm32f0x1")]
use stm32f0::stm32f0x1 as pac;
#[cfg(feature="stm32f303")]
use stm32f3::stm32f303 as pac;

#[derive(Clone, Copy, Debug)]
pub enum Channel {
    G1Ch1 = (1<<4) + 1,
    G1Ch2 = (1<<4) + 2,
    G1Ch3 = (1<<4) + 3,
    G1Ch4 = (1<<4) + 4,
    G2Ch1 = (2<<4) + 1,
    G2Ch2 = (2<<4) + 2,
    G2Ch3 = (2<<4) + 3,
    G2Ch4 = (2<<4) + 4,
    G3Ch1 = (3<<4) + 1,
    G3Ch2 = (3<<4) + 2,
    G3Ch3 = (3<<4) + 3,
    G3Ch4 = (3<<4) + 4,
    G4Ch1 = (4<<4) + 1,
    G4Ch2 = (4<<4) + 2,
    G4Ch3 = (4<<4) + 3,
    G4Ch4 = (4<<4) + 4,
    G5Ch1 = (5<<4) + 1,
    G5Ch2 = (5<<4) + 2,
    G5Ch3 = (5<<4) + 3,
    G5Ch4 = (5<<4) + 4,
    G6Ch1 = (6<<4) + 1,
    G6Ch2 = (6<<4) + 2,
    G6Ch3 = (6<<4) + 3,
    G6Ch4 = (6<<4) + 4,
    G7Ch1 = (7<<4) + 1,
    G7Ch2 = (7<<4) + 2,
    G7Ch3 = (7<<4) + 3,
    G7Ch4 = (7<<4) + 4,
    G8Ch1 = (8<<4) + 1,
    G8Ch2 = (8<<4) + 2,
    G8Ch3 = (8<<4) + 3,
    G8Ch4 = (8<<4) + 4,
}

#[derive(Clone, Copy, Debug)]
pub struct SampleConfig {
    // Groups is a u8 where bit 7 indicates it is enabled, bits 5:4 indicate the channel used
    // as the sample cap, and bits 3:0 indicate which channels are enabled for measurement.
    pub groups: [u8; 8],
}

impl SampleConfig {

    /// Create a new sample group with no groups enabled
    pub const fn new() -> Self {
        Self {
            groups: [0; 8],
        }
    }

    pub const fn sample(mut self, sample: Channel) -> Self {
        let group = ((sample as u8) >> 4) as usize;
        assert!(group > 0 && group <=8);
        let ch = (sample as u8) & 0xf;
        assert!(self.groups[group - 1] == 0);
        self.groups[group - 1] = 0x80 | ((ch as u8) << 4);
        self
    }

    pub const fn channel(mut self, channel: Channel) -> Self {
        let group = ((channel as u8) >> 4) as usize;
        assert!(group > 0 && group <=8);
        let ch = (channel as u8) & 0xf;
        assert!(self.groups[group - 1] != 0);
        let sample = (self.groups[group - 1] & 0x3) >> 4;
        assert!(sample != channel as u8);

        self.groups[group - 1] |= 1 << (ch as u8 - 1);
        self
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub clock_prescale: Option<ClockPrescaler>,
    pub max_count: Option<MaxCount>,
    pub charge_transfer_high: Option<ChargeDischargeTime>,
    pub charge_transfer_low: Option<ChargeDischargeTime>,
}

#[derive(Clone, Copy, Debug)]
pub enum ClockPrescaler {
    Hclk = 0b000,
    HclkDiv2 = 0b001,
    HclkDiv4 = 0b010,
    HclkDiv8 = 0b011,
    HclkDiv16 = 0b100,
    HclkDiv32 = 0b101,
    HclkDiv64 = 0b110,
    HclkDiv128 = 0b111,
}

#[derive(Clone, Copy, Debug)]
/// How many tsc cycles are spent charging / discharging
pub enum ChargeDischargeTime {
    C1 = 0b0000,
    C2 = 0b0001,
    C3 = 0b0010,
    C4 = 0b0011,
    C5 = 0b0100,
    C6 = 0b0101,
    C7 = 0b0110,
    C8 = 0b0111,
    C9 = 0b1000,
    C10 = 0b1001,
    C11 = 0b1010,
    C12 = 0b1011,
    C13 = 0b1100,
    C14 = 0b1101,
    C15 = 0b1110,
    C16 = 0b1111,
}

#[derive(Clone, Copy, Debug)]
pub enum MaxCount {
    /// 000: 255
    U255 = 0b000,
    /// 001: 511
    U511 = 0b001,
    /// 010: 1023
    U1023 = 0b010,
    /// 011: 2047
    U2047 = 0b011,
    /// 100: 4095
    U4095 = 0b100,
    /// 101: 8191
    U8191 = 0b101,
    /// 110: 16383
    U16383 = 0b110,
}

impl MaxCount {
    pub fn to_count(&self) -> u16 {
        match self {
            Self::U255 => 255,
            Self::U511 => 511,
            Self::U1023 => 1023,
            Self::U2047 => 2047,
            Self::U4095 => 4095,
            Self::U8191 => 8191,
            Self::U16383 => 16383,
        }
    }
}

/// Default value for charge tranfer high time if not provided
const DEFAULT_CTPH: ChargeDischargeTime = ChargeDischargeTime::C2;
/// Default value for charge tranfer low time if not provided
const DEFAULT_CTPL: ChargeDischargeTime = ChargeDischargeTime::C2;
/// Default input clock divider if not provided
const DEFAULT_PRESCALE: ClockPrescaler = ClockPrescaler::HclkDiv16;
/// Default max count setting if not provided
const DEFAULT_MAX_COUNT: MaxCount = MaxCount::U8191;

pub struct Tsc {
    tsc: &'static pac::tsc::RegisterBlock,
    max_count: u16,
}

impl Tsc {
    pub fn new(config: Option<Config>) -> Self {
        // Enable the TSC periph clock
        let rcc = unsafe { &*pac::RCC::ptr() }; // pac::Peripherals::steal().RCC };
        rcc.ahbenr.modify(|_, w| w.tscen().set_bit());
        rcc.ahbrstr.modify(|_, w| w.tscrst().set_bit());
        rcc.ahbrstr.modify(|_, w| w.tscrst().clear_bit());

        let config = config.unwrap_or(Config {
            clock_prescale: None,
            max_count: None,
            charge_transfer_high: None,
            charge_transfer_low: None,
        });

        let max_count_enum = config.max_count.unwrap_or(DEFAULT_MAX_COUNT);
        let tsc = unsafe { &*pac::TSC::ptr() }; //pac::Peripherals::steal().TSC };

        tsc.cr.write(|w| unsafe {
            w.ctph().bits(config.charge_transfer_high.unwrap_or(DEFAULT_CTPH) as u8)
            .ctpl().bits(config.charge_transfer_low.unwrap_or(DEFAULT_CTPL) as u8)
            .sse().set_bit()
            .ssd().bits(16)
            .pgpsc().bits(config.clock_prescale.unwrap_or(DEFAULT_PRESCALE) as u8)
            .mcv().bits(max_count_enum as u8)
            .tsce().set_bit()
        });

        Self {
            tsc: tsc,
            max_count: max_count_enum.to_count(),
        }
    }

    /// Setup the channel configuration and begin an acquisition
    ///
    /// sample_config: A slice of TscSample objects, each describing an enabled group
    pub fn start(&mut self, sample_config: &SampleConfig) {
        let mut iogcsr: u32 = 0;
        let mut ioscr: u32 = 0;
        let mut ioccr: u32 = 0;

        for gid in 0..sample_config.groups.len() {
            if sample_config.groups[gid] != 0 {
                let sample_id = (sample_config.groups[gid] >> 4) & 7;
                let channel_mask = sample_config.groups[gid] & 0xf;

                // Enable the group
                iogcsr |= 1 << (gid);
                // Set the sample cap input
                ioscr |= 1 << ((gid as u8) * 4 + sample_id - 1);
                // Enable the input
                ioccr |= (channel_mask as u32) << ((gid) * 4);
            }
        }

        self.tsc.iogcsr.write(|w| unsafe { w.bits(iogcsr) });
        self.tsc.ioscr.write(|w| unsafe { w.bits(ioscr) });
        self.tsc.ioccr.write(|w| unsafe { w.bits(ioccr) });

        // Start the acquisition
        self.clear_flags();
        self.tsc.cr.modify(|_, w| w.iodef().clear_bit());
        self.tsc.cr.modify(|_, w| w.start().set_bit());
    }

    /// Start an acquisition and block until it is complete
    pub fn acquire(&mut self, sample_config: &SampleConfig) {
        self.start(sample_config);
        while !self.is_finished() {}
    }

    /// Poll flags to see if acquisition has completed
    ///
    /// Returns true if the acquisition is done
    pub fn is_finished(&self) -> bool {
        let isr = self.tsc.isr.read();
        isr.eoaf().bit_is_set() || isr.mcef().bit_is_set()
    }

    /// Read the acquired value for a single group from the last acquisition
    ///
    /// If the capacitance is too low, the MaxCount value may be reached before the
    /// charge threshold. In the case, the peripheral stops acquisition and sets a
    /// flag indication the MaxCountError. Any group which reaches max count will
    /// return a value of MAX_COUNT + 1.
    ///
    /// group: The group number to read, starting at 1
    pub fn read_group(&self, group: u8) -> u16 {
        // Status bits indicate if the group completed successfully. Any group not completed
        // when MAX COUNT is reached will not be set.
        let group_status = self.tsc.iogcsr.read().bits() >> 16;

        if group_status & (1 << (group - 1)) == 0 {
            self.max_count + 1
        } else {
            match group {
                1 => self.tsc.iog1cr.read().cnt().bits(),
                2 => self.tsc.iog2cr.read().cnt().bits(),
                3 => self.tsc.iog3cr.read().cnt().bits(),
                4 => self.tsc.iog4cr.read().cnt().bits(),
                5 => self.tsc.iog5cr.read().cnt().bits(),
                6 => self.tsc.iog6cr.read().cnt().bits(),
                _ => 0,
            }
        }
    }

    /// Clear interrupt flags
    pub fn clear_flags(&mut self) {
        self.tsc.icr.write(|w| {
            w.eoaic().set_bit() // end-of-acquisition
            .mceic().set_bit() // max-count-error
        });
    }

    /// Enable both interrupts
    pub fn listen(&mut self) {
        self.tsc.ier.write(|w| {
            w.eoaie().set_bit()
            .mceie().set_bit()
        });
    }

    /// Disable both interrupts
    pub fn unlisten(&mut self) {
        self.tsc.ier.write(|w| {
            w.eoaie().clear_bit()
            .mceie().clear_bit()
        });
    }
}