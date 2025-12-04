//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use regex::Regex;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::{env, fs};

fn main() {
    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    extract_flash_info();

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");

    #[cfg(feature = "defmt")]
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}

fn extract_flash_info() {
    let mem = fs::read_to_string("memory.x")
        .expect("build.rs: failed to read memory.x from project root");

    let re_flash =
        Regex::new(r"CV_FLASH\s*:\s*ORIGIN\s*=\s*(?P<origin>.*),\s*LENGTH\s*=\s*(?P<length>.*)")
            .unwrap();

    let matches = re_flash
        .captures(&mem)
        .expect("build.rs: failed to find CV_FLASH in memory.x");

    let origin = parse_add_sub_expr(matches.name("origin").unwrap().as_str()).unwrap();
    let flash_len = parse_add_sub_expr(matches.name("length").unwrap().as_str()).unwrap();

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let out = out_dir.join("flash_consts.rs");
    fs::write(
        &out,
        format!(
            "pub const CV_FLASH_ORIGIN: usize = {};\n\
             pub const CV_FLASH_SIZE: usize = {};\n",
            origin, flash_len
        ),
    )
    .expect("build.rs: failed to write flash_consts.rs");
}

// Evaluate a simple expression with + and - of literals supporting K/M/hex/dec.
// Example: "0x10000000 + 2048K - 128K"
fn parse_add_sub_expr(s: &str) -> Option<u64> {
    let mut acc: i128 = 0;
    let mut cur = String::new();
    let mut sign: i128 = 1;

    let flush = |tok: &mut String, acc: &mut i128, sign: &mut i128| -> Option<()> {
        let t = tok.trim();
        if !t.is_empty() {
            let v = parse_size_literal(t)? as i128;
            *acc = acc.checked_add(sign.saturating_mul(v))?;
            tok.clear();
        }
        Some(())
    };

    for ch in s.chars() {
        match ch {
            '+' => {
                flush(&mut cur, &mut acc, &mut sign)?;
                sign = 1;
            }
            '-' => {
                flush(&mut cur, &mut acc, &mut sign)?;
                sign = -1;
            }
            '(' | ')' => {}
            _ => cur.push(ch),
        }
    }
    flush(&mut cur, &mut acc, &mut sign)?;
    if acc < 0 { None } else { Some(acc as u64) }
}

fn parse_size_literal(s: &str) -> Option<u64> {
    let t = s.trim();
    if let Some(num) = t.strip_suffix('K') {
        let v = parse_number(num.trim())?;
        return v.checked_mul(1024);
    }
    if let Some(num) = t.strip_suffix('M') {
        let v = parse_number(num.trim())?;
        return v.checked_mul(1024 * 1024);
    }
    parse_number(t)
}

fn parse_number(s: &str) -> Option<u64> {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).ok()
    } else {
        s.parse::<u64>().ok()
    }
}
