use std::{env, fs::File, io::Write, path::Path};

fn main() {
    println!("cargo::rerun-if-changed=memory.x");

    // Compute our fixed-point half-cycle of a sine wave.
    //
    // We use a slightly weird fixed-point scheme where 1.0 is represented
    // bitwise as 65535. This makes range analysis significantly easier in the
    // firmware.
    let half_wavetable_size = 16u16;
    let mut samples = vec![];
    for i in 0..half_wavetable_size {
        let x = f64::from(i) * std::f64::consts::PI
            / f64::from(half_wavetable_size);
        let sample = (x.sin() * 65535.).round() as u16;
        samples.push(sample);
    }

    let table_path = Path::new(&env::var("OUT_DIR").unwrap()).join("sine.rs");
    let mut table = File::create(table_path).unwrap();

    let wavetable_size = half_wavetable_size * 2;

    writeln!(table, "pub const WAVETABLE_SIZE: usize = {wavetable_size};")
        .unwrap();
    writeln!(table, "pub const DMA_SIZE: u16 = {};", wavetable_size * 2,)
        .unwrap();

    writeln!(
        table,
        "pub static COEFFICIENTS: [u16; {half_wavetable_size}] = ["
    )
    .unwrap();
    for s in samples {
        writeln!(table, "    0b{s:016b},").unwrap();
    }
    writeln!(table, "];").unwrap();
}
