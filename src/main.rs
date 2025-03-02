//! Knockoff firmware for generic AliExpress soundlaser.
//!
//! Scaled analog input arrives at PA2 (ADC_IN2).
//!
//! Ultrasound output should be delivered to PA4 (DAC_OUT1).
//!
//! PB12 and PB14 appear to be indicator LEDs. PB3 and PB5 have mysterious
//! functions I don't understand yet. PB3 does not appear to be routed, PB5 is
//! routed to what appears to be a footprint for a bluetooth module (not
//! installed).
//!
//! There are LEDs on PB12-14. The stock firmware never uses the one on PB13
//! except to briefly blink it at boot.
//!
//! No external oscillator is available in the system.
//!
//! Relevant relevant device errata:
//!
//! - 2.5.3 ADEN cannot be set immediately after ADC calibration

#![no_std]
#![no_main]

use core::{ops::Range, sync::atomic::{compiler_fence, AtomicU16, AtomicU32, Ordering}};

use panic_halt as _;
use stm32_metapac::{self as pac, flash::vals::Latency, gpio::vals::{Moder, Ospeedr, Pupdr}, interrupt, iwdg::vals::Key, rcc::vals::{Hpre, Pllmul, Pllsrc, Ppre, Sw}};

mod generated {
    include!(concat!(env!("OUT_DIR"), "/sine.rs"));
}
use generated::WAVETABLE_SIZE;
use generated::COEFFICIENTS;

/// Intended carrier frequency for the ultrasonic wave. In practice, this will
/// be approximated, though we'll get as close as we can.
const CARRIER_FREQ: usize = 40_000;

/// CPU frequency in Hz. If you change this, you must update the
/// `configure_clock_tree` routine to match, or you'll get the wrong output
/// frequency.
const CPU_FREQ: usize = 48_000_000;

/// The middle of the ADC and DAC ranges, used as the "zero" point for incoming
/// and outgoing waves. Do not change this, it's intrinsic.
const MIDPOINT: u16 = (u16::MAX / 2) + 1;

/// Our RAM wavetable buffer, filled in from the DMA ISR. We have two copies of
/// the wavetable so we can update one while the other is streaming. This uses
/// atomics to ensure that all accesses avoid tearing when racing DMA. It also
/// makes access to the static array more convenient (i.e. not unsafe).
static WAVETABLE: [[AtomicU16; WAVETABLE_SIZE]; 2] =
    [const { [const { AtomicU16::new(0) }; WAVETABLE_SIZE] }; 2];

/// Firmware entry point and main loop.
#[cortex_m_rt::entry]
fn main() -> ! {
    // To avoid glitching the output to full-negative for one or two wave
    // cycles, go ahead and initialize the wavetable.
    //
    // Yes, this arguably duplicates the setup done before `main`. We could have
    // written `WAVETABLE` with an initializer that sets each element to
    // `MIDPOINT`. However, because memory initialization in rustc/llvm is
    // pretty naive, this produces a complete initialized copy of `WAVETABLE` in
    // flash at a cost of (currently) 256 bytes.
    //
    // Leaving it in .bss and initializing it with a loop is dramatically
    // cheaper (about 25% the cost).
    for half in &WAVETABLE {
        for sample in half {
            sample.store(MIDPOINT, Ordering::Relaxed);
        }
    }

    // Hardware initialization: "steal" the peripherals to defeat cortex_m's
    // ownership model. This avoids a bunch of code generated for no reason.
    //
    // Safety: so tbh the cortex_m crate abuses the definition of safety here to
    // enforce its own ideas about peripheral use. That being said, this is even
    // safe in their sense of the term, because it happens exactly once at the
    // top of main.
    let mut cp = unsafe { cortex_m::Peripherals::steal() };

    // This needs to happen first:
    configure_clock_tree();
    allow_watchdog_freeze_on_halt();

    // The order of these steps is essentially arbitrary, except that it's nice
    // to configure GPIO _after_ DAC so we don't briefly expose a nonsense
    // value, but _before_ ADC so it doesn't see a floating value.
    configure_dac();
    configure_gpios();
    configure_adc();
    configure_dma();

    // This should happen after any initialization that might spin (e.g. ADC
    // calibration, clock setup) but _before_ any output.
    //
    // TODO: ...so should the DAC go after this then
    if !cfg!(feature = "disable-iwdg") {
        // Start the watchdog. At this point if we don't start generating
        // waveforms soon, we will reset. (The watchdog is fed from the DMA
        // progress ISR.)
        configure_iwdg();
    }

    // Unmask our DMA interrupt, which is where most of the actual behavior is
    // implemented.
    //
    // Safety: this is unsafe if it might break a critical section by allowing
    // preemption by the ISR, but we don't actually share any data with the ISR,
    // so it can preempt us -- go right ahead.
    unsafe {
        cp.NVIC.set_priority(pac::Interrupt::DMA1_CHANNEL2_3, 0);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_CHANNEL2_3);
    }

    // This is what kicks off the actual scanout:
    configure_sample_timer();

    loop {
        // The original firmware monitored PB3 to provide some sort of
        // output-disable function. I have not reproduced this here because it's
        // weird.

        cortex_m::asm::wfi();
    }
}

/// Interrupt service routine triggered twice during each scanout of the
/// wavetable: once at the halfway point, and once at the end.
#[interrupt]
fn DMA1_CHANNEL2_3() {
    let isr = pac::DMA1.isr().read();

    if isr.tcif(3 - 1) {
        // Clear DMA CH1 TC flag.
        pac::DMA1.ifcr().write(|w| {
            w.set_tcif(3 - 1, true);
        });
        // DMA has just finished the second copy of the wavetable, so regenerate
        // it.
        regenerate_waveform(1);
    } else if isr.htif(3 - 1) {
        // Clear DMA CH1 TC flag.
        pac::DMA1.ifcr().write(|w| {
            w.set_htif(3 - 1, true);
        });
        // DMA has just finished the _first_ copy of the wavetable.
        regenerate_waveform(0);
    }
}

/// Fills in `WAVEFORM[which]` with a new pattern based on the current ADC
/// level.
fn regenerate_waveform(which: usize) {
    // Size of band around zero that we treat as "quiet," for the purpose of
    // suppressing the carrier signal.
    const DEADZONE: u16 = 2400;
    const ZERO_RANGE: Range<u16> = MIDPOINT - DEADZONE..MIDPOINT + DEADZONE;

    // We update this variable each time through the routine, to keep track of
    // how long the input has been "quiet."
    static QUIET_TIME: AtomicU32 = AtomicU32::new(0);

    // Light the middle LED to help indicate how much CPU time is being spent
    // doing this.
    pac::GPIOB.bsrr().write(|w| w.set_bs(13, true));

    // Read the most recent ADC sample. The ADC updates at around 144 kHz, so
    // this will be at most 6.94 µs old. (We produce one cycle of output every
    // 25 µs so this is "recent enough.")
    //
    // Note that this is a left-aligned 12-bit sample (0..=0xFFF0).
    let sample = pac::ADC1.dr().read().data();

    // Increment our quiet-time counter if the sample is close to the midpoint.
    let quiet_time_now = if !ZERO_RANGE.contains(&sample) {
        0
    } else {
        QUIET_TIME.load(Ordering::Relaxed).saturating_add(1)
    };
    QUIET_TIME.store(quiet_time_now, Ordering::Relaxed);

    if quiet_time_now < 0x1200 {
        let sample = u32::from(sample);

        // Update the waveform. Since we're generating a sine wave, the two
        // halves are symmetrical, and we can generate them at the same time by
        // applying different offsets from the midpoint.
        //
        // First, break the waveform table in half:
        let (pos_cycle, neg_cycle) = WAVETABLE[which].split_at(COEFFICIENTS.len());
        // Now, process the two halves and the coefficient table in parallel.
        for ((outp, outn), &coeff) in pos_cycle.iter()
            .zip(neg_cycle)
            .zip(&COEFFICIENTS)
        {
            // Promote both sides to u32 so we can do a 16x16->32
            // multiplication, and then take the top half, divided by 2.
            //
            // We could remove the "divided by 2" part by halving the values in
            // the COEFFICIENTS table, but using the full range of u16 makes the
            // range analysis easier on the compiler, and "top half" and "top
            // half divided by 2" are the same cost on ARMv6-M.
            let x = ((sample * u32::from(coeff)) >> 17) as u16;

            // Range of x:
            //
            // - coeff is in the range 0..=0xFFFF (by definition of u16)
            // - sample is in the range 0..=0xFFF0 (12-bit left-aligned)
            // - (coeff * sample) is thus in the range 0..=0xfffe_0001
            // - thus x is in the range 0..=0x7fff.
            //
            // Since we're relying on the ranges of the types, and not the
            // specific values of the COEFFICIENTS table, the compiler
            // recognizes this and the addition/subtraction below do not
            // generate overflow checks.

            // Generate two samples on opposite sides of our midpoint.
            outp.store(MIDPOINT + x, Ordering::Relaxed);
            outn.store(MIDPOINT - x, Ordering::Relaxed);
        }
        // Switch the state of the indicator lights to show that we're producing
        // a carrier.
        pac::GPIOB.bsrr().write(|w| {
            w.set_bs(12, true);
            w.set_br(14, true);
        });
    } else {
        // Blank the stored waveform to stop producing a carrier.
        for sample in &WAVETABLE[which] {
            sample.store(MIDPOINT, Ordering::Relaxed);
        }
        // Switch the state of the indicator lights.
        pac::GPIOB.bsrr().write(|w| {
            w.set_br(12, true);
            w.set_bs(14, true);
        });
    }

    if !cfg!(feature = "disable-iwdg") {
        // Reassure the watchdog timer that we're making forward progress.
        feed_iwdg();
    }

    // Turn off the middle LED to show that we're done.
    pac::GPIOB.bsrr().write(|w| w.set_br(13, true));
}

/// Sets up the simplest practical clock tree for full-speed operation.
fn configure_clock_tree() {
    // Our intent is to run the CPU and all peripherals at 48 MHz.
    //
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

/// Configures all GPIOs from their reset states (high-impedance inputs) to the
/// states required for this application.
///
/// This routine is also responsible for turning on clock to the GPIO ports, so
/// it's important to call this before any GPIO manipulation.
fn configure_gpios() {
    // Turn on clocks to the GPIO ports.
    pac::RCC.ahbenr().modify(|w| {
        w.set_gpioaen(true);
        w.set_gpioben(true);
    });
    compiler_fence(Ordering::SeqCst);

    // GPIOA: pins PA2 and PA4 in analog mode.
    pac::GPIOA.moder().modify(|w| {
        w.set_moder(2, Moder::ANALOG);
        w.set_moder(4, Moder::ANALOG);
    });

    // GPIOB: pins PB12, PB13, PB14 in push-pull fast output mode.
    //
    // PB5 is initially an input with a pullup.

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
    pac::GPIOB.pupdr().modify(|w| {
        w.set_pupdr(5, Pupdr::PULL_UP);
    });
    pac::GPIOB.moder().modify(|w| {
        for pin in 12..=14 {
            w.set_moder(pin, Moder::OUTPUT);
        }
        // This should be default at reset, but, hey
        w.set_moder(5, Moder::INPUT);
    });

}

/// Sets up DMA transfer from memory to the DAC. Also responsible for enabling
/// the clock to the DMA controller.
fn configure_dma() {
    // This implementation uses a single DMA channel. The F0 has probably the
    // simplest DMA implementation of the STM32 family, and in particular, has
    // no channel DRQ mux: DRQs are just OR'ed into channels. So, we have a lot
    // of constraints on which channel we use for what.
    //
    // CH3 (DAC) moves samples from the waveform table to the DAC. It's
    // configured for M-2-P, MINC, circular. The DRQs to this channel are being
    // driven on the DAC DRQ, but the DAC is driving its DRQ in response to
    // TIM2 behind the scenes. See the timer setup routine for more.
    
    // Enable clock to the DMA controller.
    pac::RCC.ahbenr().modify(|w| w.set_dmaen(true));
    compiler_fence(Ordering::SeqCst);

    let dma = pac::DMA1;

    {
        let ch3 = dma.ch(3 - 1);
        ch3.cr().write(|w| {
            w.set_dir(pac::bdma::vals::Dir::FROM_MEMORY);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_msize(pac::bdma::vals::Size::BITS16);
            w.set_psize(pac::bdma::vals::Size::BITS32);
            w.set_circ(true);
            w.set_pl(pac::bdma::vals::Pl::HIGH);
        });
        // We stream out both copies of the wavetable.
        ch3.ndtr().write(|w| w.set_ndt(generated::DMA_SIZE));
        // Note that we're choosing the left-aligned version here.
        ch3.par().write_value(pac::DAC1.dhr12l(0).as_ptr() as u32);
        ch3.mar().write_value(WAVETABLE.as_ptr() as u32);

        // Enable both the half-transfer and transfer-complete interrupts.
        ch3.cr().modify(|w| {
            w.set_tcie(true);
            w.set_htie(true);
        });

        // Turn CH3 on.
        ch3.cr().modify(|w| w.set_en(true));
    }
}

/// Sets up the analog-to-digital converter, which we configure to free-running
/// mode doing 12-bit conversions.
fn configure_adc() {
    // Enable clock to the ADC.
    pac::RCC.apb2enr().modify(|w| {
        w.set_adcen(true);
    });
    compiler_fence(Ordering::SeqCst);

    let adc = pac::ADC1;

    adc.cfgr1().modify(|w| {
        // Continuous conversion
        w.set_cont(true);
        // Left-align the 12-bit sample in the 16-bit field, producing a value
        // in the range 0..=0xFFF0. This makes some math downstream more
        // convenient.
        w.set_align(pac::adc::vals::Align::LEFT);

        // Defaults restated here for clarity.
        w.set_res(pac::adc::vals::Res::BITS12);
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

    // This next bit is written in response to device erratum 2.5.3, "ADEN bit
    // cannot be set immediately after the ADC calibration." Calibration will
    // clear ADEN 4 cycles after it completes, so if the compiler puts our store
    // to CR too close to the checks above, it could get negated.
    //
    // This is super unlikely, because a RMW on a Cortex-M0 is at least four
    // cycles. But still, let's apply the workaround suggested by ST: repeatedly
    // set ADEN from within the ADRDY wait loop.
    while !adc.isr().read().adrdy() {
        // Enable the ADC (again?)
        adc.cr().modify(|w| w.set_aden(true));
    }

    // Start it!
    adc.cr().modify(|w| w.set_adstart(true));
}

/// Configures the DAC itself, which is responsible for driving DMA, and is in
/// turn driven by TIM2 (so this won't do much if you don't also call
/// `configure_sample_timer`).
fn configure_dac() {
    // Enable clock to the DAC.
    pac::RCC.apb1enr().modify(|w| {
        w.set_dacen(true);
    });
    compiler_fence(Ordering::SeqCst);

    let dac = pac::DAC1;

    dac.cr().write(|w| {
        // Disable output buffer.
        //
        // This keeps us from overdriving the output stage.
        w.set_boff(0, true);
        // Enable external trigger.
        w.set_ten(0, true);
        // Source external trigger from TIM2 TRGO. This is how the DMA gets
        // driven from TIM2.
        w.set_tsel(0, 0b100);
        // Enable DRQ generation.
        w.set_dmaen(0, true);
    });
    // Enable it.
    dac.cr().modify(|w| w.set_en(0, true));
    // Initialize the output to the midpoint value. The reference manual is
    // ambiguous on whether this can be done before the channel is enabled, so
    // we'll do it after, even though that implies a very brief output glitch.
    dac.dhr12l(0).write(|w| w.set_dhr(MIDPOINT));
}

/// Configures TIM2 to drive the DAC to produce samples at our target rate.
fn configure_sample_timer() {
    // This configures TIM2 to scan out our wavetable at `TARGET_FREQ`.
    //
    // TIM2's input clock is the same as `CPU_FREQ` in our configuration.
    const CYCLES_PER_WAVE: usize = CPU_FREQ / CARRIER_FREQ;
    // Round the calculation to minimize error (computes a slightly different
    // carrier frequency than the original).
    const CYCLES_PER_SAMPLE: usize =
        (CYCLES_PER_WAVE + WAVETABLE_SIZE/2) / WAVETABLE_SIZE;
    // The timer wants the period N set as ARR=N-1; compute N-1 at compile time
    // so the compiler will error out if we happen to underflow.
    const SETTING: u32 = (CYCLES_PER_SAMPLE - 1) as u32;
    
    // Enable clock to TIM2.
    pac::RCC.apb1enr().modify(|w| w.set_tim2en(true));
    compiler_fence(Ordering::SeqCst);

    let tim = pac::TIM2;
    tim.arr().write_value(SETTING);
    tim.psc().write_value(0);
    // Force double-buffered registers to be applied.
    tim.egr().write(|w| w.set_ug(true));
    // Generate TRGO on UPDATE events (excluding the fake one we just
    // generated). This is the signal that the DAC will respond to; see the DAC
    // configuration.
    tim.cr2().write(|w| {
        w.set_mms(stm32_metapac::timer::vals::Mms::UPDATE);
    });
    // Enable the timer.
    tim.cr1().write(|w| w.set_cen(true));
}

/// Sets up the Independent Watchdog to reset the processor if we hang, ensuring
/// that we don't just sit there and blast the amp or bystanders.
fn configure_iwdg() {
    // The IWDG is one of the only peripherals (along with RCC and FLASH) that
    // can be used _without_ having to enable its clock in RCC.
    let iwdg = pac::IWDG;

    // The original firmware set the IWDG with a prescaler of 4 and a period of
    // 625. The IWDG is clocked from the LSI oscillator, at a speed of about 40
    // kHz. That gives it a timeout of 62.5 ms.
    //
    // This seems absurdly high, given that under normal operation the original
    // firmware feeds the watchdog at 144 kHz. This implementation feeds the
    // watchdog once per output cycle, or about 40 kHz.

    // Perform the watchdog register unlock sequence.
    iwdg.kr().write(|w| w.set_key(Key::ENABLE));

    // Configure timing.
    iwdg.pr().write(|w| w.set_pr(pac::iwdg::vals::Pr::DIVIDE_BY4));
    iwdg.rlr().write(|w| w.set_rl(625));

    // Initialize the counter. Failing to do this here causes the watchdog to
    // start the first countdown from 0xFFF, giving us 409.5 ms of leniency on
    // boot. We don't need that.
    iwdg.kr().write(|w| w.set_key(Key::RESET));
    // And start the watchdog. This operation cannot be reversed (without a
    // reset).
    iwdg.kr().write(|w| w.set_key(Key::START));
}

/// Reloads the IWDG countdown timer, postponing our inevitable demise.
fn feed_iwdg() {
    pac::IWDG.kr().write(|w| w.set_key(Key::RESET));
}

/// Halt the IWDG on debug halt. This makes it much easier to use a debugger.
/// Yes, probe software can do this, and OpenOCD does, but then there's probe-rs
/// which decided not to make anything configurable.
fn allow_watchdog_freeze_on_halt() {
    // Enable access to the DBGMCU unit.
    pac::RCC.apb2enr().modify(|w| w.set_dbgmcuen(true));
    compiler_fence(Ordering::SeqCst);

    // Stop the watchdog on debug halt.
    pac::DBGMCU.apb1_fz().modify(|w| w.set_iwdg(true));
    // Note that we are _not_ freezing the DAC/TIM2 on halt.
}
