//! Knockoff firmware for generic AliExpress soundlaser.
//!
//! Scaled analog input arrives at PA2 (ADC_IN2).
//!
//! Ultrasound output should be delivered to PA4 (DAC_OUT1).
//!
//! PB12 and PB14 appear to be indicator LEDs. PB5 has a mysterious function I
//! don't understand yet.
//!
//! No oscillator is available in the system.
//!
//! Potentially relevant device errata:
//!
//! - 2.5.3 ADEN cannot be set immediately after ADC calibration

#![no_std]
#![no_main]

use core::{ops::Range, ptr::addr_of, sync::atomic::Ordering};
use portable_atomic::AtomicU32;

use panic_halt as _;
use stm32_metapac::{self as pac, interrupt, flash::vals::Latency, gpio::vals::{Moder, Ospeedr}, rcc::vals::{Hpre, Pllmul, Pllsrc, Ppre, Sw}};

const WAVETABLE_SIZE: u16 = 32;
const TARGET_FREQ: u32 = 40_000;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();

    configure_clock_tree();
    configure_gpios();
    configure_adc();
    configure_dac();
    configure_dma();

    // TODO setup IWDG

    // Unmask our DMA transfer complete interrupt, which is where most of the
    // actual behavior is implemented.
    //
    // Safety: this is unsafe if it might break a critical section by allowing
    // preemption by the ISR, but we don't actually share any data with the ISR,
    // so it can preempt us -- go right ahead.
    unsafe {
        cp.NVIC.set_priority(pac::Interrupt::DMA1_CHANNEL1, 0);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_CHANNEL1);
    }

    // This is what kicks off the actual scanout:
    configure_sample_timer();

    loop {
        // The original firmware monitored PB3 to provide some sort of
        // output-disable function. I have not reproduced this here because it's
        // weird.

        core::sync::atomic::compiler_fence(Ordering::SeqCst);
    }
}

#[interrupt]
fn DMA1_CHANNEL1() {
    static QUIET_TIME: AtomicU32 = AtomicU32::new(0);

    // Read the last sample.
    //
    // Safety: we're racing the DMA controller, which is deliberate. Both we and
    // the DMA controller will make atomic accesses to the variable, so we can't
    // get tearing, only racing.
    let sample = unsafe {
        addr_of!(LAST_SAMPLE).read_volatile()
    };

    const ZERO_RANGE: Range<u16> = 0x76d..0x898;

    let quiet_time_now = if !ZERO_RANGE.contains(&sample) {
        QUIET_TIME.store(0, Ordering::Relaxed);
        0
    } else {
        QUIET_TIME.fetch_add(1, Ordering::Relaxed)
    };

    if quiet_time_now < 0x1200 {
        // Update the waveform.
        let sample = u32::from(sample);
        let (first_half, second_half) = WAVETABLE.split_at(COEFFICIENTS.len());
        for (out, coeff) in first_half.iter().zip(COEFFICIENTS) {
            let x = 0x7ff + (sample.wrapping_mul(coeff) >> 16);
            out.store(x, Ordering::Relaxed);
        }
        for (out, coeff) in second_half.iter().zip(COEFFICIENTS) {
            let x = 0x7ff - (sample.wrapping_mul(coeff) >> 16);
            out.store(x, Ordering::Relaxed);
        }
        // Switch the state of the indicator lights.
        pac::GPIOB.bsrr().write(|w| {
            w.set_bs(12, true);
            w.set_br(14, true);
        });
        // Turn DAC DRQ back on just in case we turned it off.
        pac::DAC1.cr().modify(|w| w.set_dmaen(0, true));
    } else {
        // Blank the stored waveform.
        for sample in &WAVETABLE {
            sample.store(0, Ordering::Relaxed);
        }
        // Shut off DAC DRQ generation.
        pac::DAC1.cr().modify(|w| w.set_dmaen(0, false));
        // Switch the state of the indicator lights.
        pac::GPIOB.bsrr().write(|w| {
            w.set_br(12, true);
            w.set_bs(14, true);
        });
    }

    // Clear DMA CH1 TC flag.
    pac::DMA1.ifcr().write(|w| {
        w.set_tcif(0, true);
    });

    // TODO feed IWDG
}

fn configure_clock_tree() {
    // There is no external oscillator installed, so we need to use HSI through
    // a PLL to get 48 MHz.
    //
    // We come out of reset on HSI at 8 MHz, so that's relatively easy.
    //
    // The PLL src on the F051 has two options: HSI/2, and HSE/PREDIV. So we'll
    // use HSI/2 then, for a 4 MHz input clock.
    //
    // Getting 48 MHz from this requires multiplying by 12.
    let rcc = pac::RCC;
    let flash = pac::FLASH;

    // First, configure the flash controller to use appropriate wait states for
    // 48 MHz.
    flash.acr().write(|w| {
        w.set_latency(Latency::WS1);
        // Turn on the prefetch buffer to make that a bit less painful.
        w.set_prftbe(true);
    });

    // Now, apply settings to the PLL.
    rcc.cfgr().write(|w| {
        w.set_pllmul(Pllmul::MUL12);
        w.set_pllsrc(Pllsrc::HSI_DIV2);

        // Run the downstream bus clocks at the full system rate. (This is
        // already true at reset, but I'm restating it here so when I come
        // looking for it later, I see this code.)
        w.set_ppre(Ppre::DIV1);
        w.set_hpre(Hpre::DIV1);
    });

    // Turn the PLL on.
    rcc.cr().modify(|w| w.set_pllon(true));
    // Wait for it to lock.
    while !rcc.cr().read().pllrdy() {
        // spin
    }

    // Switch to PLL as system clock source.
    rcc.cfgr().modify(|w| w.set_sw(Sw::PLL1_P));
    // Wait for this to be acknowledged.
    while rcc.cfgr().read().sws() != Sw::PLL1_P {
        // spin
    }
}

fn configure_gpios() {
    // Turn on clocks to the GPIO ports.
    pac::RCC.ahbenr().modify(|w| {
        w.set_gpioaen(true);
        w.set_gpioben(true);
    });
    core::sync::atomic::compiler_fence(Ordering::SeqCst);

    // GPIOA: pins PA2 and PA4 in analog mode.
    pac::GPIOA.moder().modify(|w| {
        w.set_moder(2, Moder::ANALOG);
        w.set_moder(4, Moder::ANALOG);
    });

    // GPIOB: pins PB12, PB13, PB14 in push-pull fast output mode.
    // Note: function of PB13 not currently understood.
    //
    // Pins start initially low.
    pac::GPIOB.bsrr().write(|w| {
        for pin in 12..=14 {
            w.set_br(pin, true);
        }
    });
    pac::GPIOB.ospeedr().modify(|w| {
        for pin in 12..=14 {
            w.set_ospeedr(pin, Ospeedr::VERY_HIGH_SPEED);
        }
    });
    pac::GPIOB.moder().modify(|w| {
        for pin in 12..=14 {
            w.set_moder(pin, Moder::OUTPUT);
        }
    });

}

fn configure_dma() {
    // This implementation uses two DMA channels. The F0 has probably the
    // simplest DMA implementation of the STM32 family, and in particular, has
    // no channel DRQ mux: DRQs are just OR'ed into channels. So, we have a lot
    // of constraints on which channel we use for what.
    //
    // CH1 (ADC DRQ) moves samples from the ADC into RAM. It's configured for
    // P-2-M, no increment (sort of), circular.
    //
    // CH3 (DAC) moves samples from the waveform table to the DAC. It's
    // configured for M-2-P, MINC, circular. Counter-intuitively, this DMA
    // channel is not directly driven by the DAC timer TIM2, but rather TIM2
    // drives the DAC behind the scenes. See the timer setup for more.
    
    // Enable clock to the DMA controller.
    pac::RCC.ahbenr().modify(|w| w.set_dmaen(true));
    core::sync::atomic::compiler_fence(Ordering::SeqCst);

    let dma = pac::DMA1;

    {
        let ch1 = dma.ch(0 /* 1 - 1 */);
        ch1.cr().write(|w| {
            w.set_dir(pac::bdma::vals::Dir::FROM_PERIPHERAL);
            // Oddly, the original firmware sets MINC. But the channel is set
            // circular, so this doesn't matter. We'll reproduce this for now
            // (TODO).
            w.set_minc(true);
            w.set_pinc(false);
            w.set_msize(pac::bdma::vals::Size::BITS16);
            w.set_psize(pac::bdma::vals::Size::BITS16);
            w.set_circ(true);
            w.set_pl(pac::bdma::vals::Pl::MEDIUM);
        });
        // Transfer one sample, over and over.
        ch1.ndtr().write(|w| w.set_ndt(1));
        ch1.par().write(|w| {
            *w = pac::ADC1.dr().as_ptr() as u32;
        });
        ch1.mar().write(|w| {
            *w = addr_of!(LAST_SAMPLE) as u32;
        });

        // Turn CH1 on.
        ch1.cr().modify(|w| w.set_en(true));

        // Enable transfer-complete interrupt.
        ch1.cr().modify(|w| w.set_tcie(true));
    }

    {
        let ch3 = dma.ch(3 - 1);
        ch3.cr().write(|w| {
            w.set_dir(pac::bdma::vals::Dir::FROM_MEMORY);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_msize(pac::bdma::vals::Size::BITS32);
            w.set_psize(pac::bdma::vals::Size::BITS32);
            w.set_circ(true);
            w.set_pl(pac::bdma::vals::Pl::HIGH);
        });
        ch3.ndtr().write(|w| w.set_ndt(WAVETABLE_SIZE));
        ch3.par().write(|w| {
            *w = pac::DAC1.dhr12r(0).as_ptr() as u32;
        });
        ch3.mar().write(|w| {
            *w = WAVETABLE.as_ptr() as u32;
        });

        // Turn CH3 on.
        ch3.cr().modify(|w| w.set_en(true));
    }

}

fn configure_adc() {
    // Enable clock to the ADC.
    pac::RCC.apb2enr().modify(|w| {
        w.set_adcen(true);
    });
    core::sync::atomic::compiler_fence(Ordering::SeqCst);

    let adc = pac::ADC1;

    adc.cfgr1().modify(|w| {
        // Continue generating DRQs even after last channel is read out.
        w.set_dmacfg(pac::adc::vals::Dmacfg::CIRCULAR);
        // Continuous conversion
        w.set_cont(true);
        // Weirdly, the original firmware sets scan direction to backwards,
        // despite enabling only a single channel. TODO: try removing
        w.set_scandir(pac::adc::vals::Scandir::BACKWARD);

        // Defaults restated here for clarity.
        w.set_align(pac::adc::vals::Align::RIGHT);
        w.set_res(pac::adc::vals::Res::BITS12);
    });

    adc.cfgr1().modify(|w| {
        w.set_dmaen(true);
    });

    adc.chselr().write(|w| {
        // Enable ADC_IN2 as the only channel scanned.
        w.set_chsel_x(2, true);
    });

    // Run a calibration. Not clear this is necessary, but, the original does
    // it! TODO: try removing
    adc.cr().write(|w| w.set_adcal(true));
    // Wait for hardware to clear the bit.
    while adc.cr().read().adcal() {
        // spin
    }

    // Enable the ADC.
    adc.cr().modify(|w| w.set_aden(true));
    // Wait for it to be ready.
    while !adc.isr().read().adrdy() {
        // spin
    }

    // Start it!
    adc.cr().modify(|w| w.set_adstart(true));
}

fn configure_dac() {
    // Enable clock to the DAC.
    pac::RCC.apb1enr().modify(|w| {
        w.set_dacen(true);
    });
    core::sync::atomic::compiler_fence(Ordering::SeqCst);

    let dac = pac::DAC1;

    dac.cr().write(|w| {
        // Disable output buffer.
        w.set_boff(0, true);
        // Enable external trigger.
        w.set_ten(0, true);
        // Source external trigger from TIM2 TRGO
        w.set_tsel(0, 0b100);
        // Enable DRQ generation.
        w.set_dmaen(0, true);
    });
    // Enable it.
    dac.cr().modify(|w| w.set_en(0, true));
}

fn configure_sample_timer() {
    // This configures TIM2 to scan out our wavetable at roughly 44 kHz.
    //
    // TIM2's input clock is PCLK, which we've set to SYSCLK, which is 48 MHz.
    const DIVISOR: u32 = (48_000_000 / WAVETABLE_SIZE as u32 + TARGET_FREQ/2) / TARGET_FREQ;
    const SETTING: u32 = DIVISOR - 1; // ensure it's non-zero at compile time.
    
    // Enable clock to TIM2.
    pac::RCC.apb1enr().modify(|w| w.set_tim2en(true));
    core::sync::atomic::compiler_fence(Ordering::SeqCst);

    let tim = pac::TIM2;
    tim.arr().write(|w| *w = SETTING);
    tim.psc().write(|w| *w = 0);
    // Force double-buffered registers to be applied.
    tim.egr().write(|w| w.set_ug(true));
    // Generate TRGO on UPDATE events (excluding the fake one we just generated)
    tim.cr2().write(|w| {
        w.set_mms(stm32_metapac::timer::vals::Mms::UPDATE);
    });
    // Enable the timer.
    tim.cr1().write(|w| w.set_cen(true));
}

static WAVETABLE: [AtomicU32; WAVETABLE_SIZE as usize] = [const { AtomicU32::new(0) }; WAVETABLE_SIZE as usize];
static mut LAST_SAMPLE: u16 = 0;

const COEFFICIENTS: [u32; WAVETABLE_SIZE as usize / 2] = [
    0,
    (1 << (16 - 4)) + (1 << (16 - 5)) + (1 << (16 - 8)),
    (1 << (16 - 3)) + (1 << (16 - 4)) + (1 << (16 - 8)),
    (1 << (16 - 2)) + (1 << (16 - 6)) + (1 << (16 - 7)) + (1 << (16 - 8)),
    (1 << (16 - 2)) + (1 << (16 - 4)) + (1 << (16 - 5)) + (1 << (16 - 7)) + (1 << (16 - 9)),
    (1 << (16 - 2)) + (1 << (16 - 3)) + (1 << (16 - 5)) + (1 << (16 - 7)) + (1 << (16 - 10)),
    (1 << (16 - 2)) + (1 << (16 - 3)) + (1 << (16 - 4)) + (1 << (16 - 6)) + (1 << (16 - 7)) + (1 << (16 - 10)),
    (1 << (16 - 2)) + (1 << (16 - 3)) + (1 << (16 - 4)) + (1 << (16 - 5)) + (1 << (16 - 6)) + (1 << (16 - 8)) + (1 << (16 - 9)),
    (1 << (16 - 1)),
    (1 << (16 - 2)) + (1 << (16 - 3)) + (1 << (16 - 4)) + (1 << (16 - 5)) + (1 << (16 - 6)) + (1 << (16 - 8)) + (1 << (16 - 9)),
    (1 << (16 - 2)) + (1 << (16 - 3)) + (1 << (16 - 4)) + (1 << (16 - 6)) + (1 << (16 - 7)) + (1 << (16 - 10)),
    (1 << (16 - 2)) + (1 << (16 - 3)) + (1 << (16 - 5)) + (1 << (16 - 7)) + (1 << (16 - 10)),
    (1 << (16 - 2)) + (1 << (16 - 4)) + (1 << (16 - 5)) + (1 << (16 - 7)) + (1 << (16 - 9)),
    (1 << (16 - 2)) + (1 << (16 - 6)) + (1 << (16 - 7)) + (1 << (16 - 8)),
    (1 << (16 - 3)) + (1 << (16 - 4)) + (1 << (16 - 8)),
    (1 << (16 - 4)) + (1 << (16 - 5)) + (1 << (16 - 8)),
];
