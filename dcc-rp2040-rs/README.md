# RP2040 based DCC decoder in Rust

## Flashing instructions

```sh
probe-rs erase --chip RP2040 --protocol swd

cd bootloader
cargo flash --release --no-default-features
```

## Development
Build and run with probe-rs (from `application`):
```sh
cargo run --release
```

SWD commonly becomes disconnected due to noise. To prevent firmware hangs, `defmt-rtt ` uses the `disable-blocking-mode`
feature.
To reconnect, run (from `application`):
```sh
probe-rs attach --chip RP2040 --protocol swd ../target/thumbv6m-none-eabi/release/DCC-RP2040-rs
```


### Debugging

PIN Layout, from the "Rev. 0.3" (outside / rightmost) end:

| Pad  | Signal  |
|---|---|
| J18  | SWCLK (Right)  |
| J19  | SWDIO (Middle)  |
| J20 | RESET (Left) |

### JLinkGDBServerCLExe
In Clion create a new embedded gdb server run configuration that uses:
- Target remote args: `tcp::17211`
- GDB Server: `/usr/bin/JLinkGDBServerCLExe`
- GDB Server Args: `-device "RP2040" -if swd -speed 8000 -port 17211 -nogui -singlerun`
- Before Launch: `cargo flash target`

### ProbeRs
Seems that it doesnt like to reset the chip properly, so doesn't work well for early debugging

```shell
probe-rs gdb --chip RP2040
```

Then, in clion create a new embedded gdb server run configuration that uses:
- Target Remote Args: `:1337`.
- GDB Server: `probe-rs`
- GDB Server Args: `gdb --chip RP2040`
- Before Launch: `cargo flash target`
