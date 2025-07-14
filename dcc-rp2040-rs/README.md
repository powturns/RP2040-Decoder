# RP2040 based DCC decoder in Rust

## Flashing instructions

```sh
# if probe doesnt respond, try increasing source voltage (vbat) to 3.7

probe-rs erase --chip nRF52840_xxAA --protocol swd

cd bootloader
cargo flash --release --no-default-features
```


## Debugging

### JLinkGDBServerCLExe
In Clion create a new embedded gdb server run configuration that uses:
- Target remote args: `tcp::17211`
- GDB Server: `/usr/bin/JLinkGDBServerCLExe`
- GDB Server Args: `-device "nRF52840_xxAA" -if swd -speed 8000 -port 17211 -nogui -singlerun`
- Before Launch: `cargo flash target`

### ProbeRs
Seems that it doesnt like to reset the chip properly, so doesn't work well for early debugging

```shell
probe-rs gdb --chip nRF52840_xxAA
```

Then, in clion create a new embedded gdb server run configuration that uses:
- Target Remote Args: `:1337`.
- GDB Server: `probe-rs`
- GDB Server Args: `gdb --chip nRF52840_xxAA`
- Before Launch: `cargo flash target`
