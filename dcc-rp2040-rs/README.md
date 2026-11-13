# RP2040 based DCC decoder in Rust

## User Guide
### Flashing instructions

```sh
probe-rs erase --chip RP2040 --protocol swd

cd bootloader
cargo flash --release --no-default-features
```

### Motor calibration

The speed table maps each speed step to a back-EMF (BEMF) target: step 126 (max) targets `CV5 × 16` ADC counts and
step 63 (mid) targets `CV6 × 16`. If CV5 is higher than the BEMF the motor can actually reach, the top of the throttle
range is flat; if it is lower, the loco never reaches its full speed. The calibration run measures the BEMF the
motor reaches at 100% PWM on your track voltage and sets CV5 and CV6 from it.

#### Procedure

The run is started with Programming on the Main (POM / operations mode programming). It cannot be started on the
programming track, which can't supply enough current to run the motor.

1. Put the loco on the main track with about 2 m of clear, straight, level track in front of it, or on a roller
   stand. The loco runs **forward at full speed** for about 1.6 s.
2. Select the loco and make sure the throttle is at 0. The run is refused if the loco is moving.
3. Optional: write **CV179** (headroom) by POM if you don't want the default. See below.
4. Write **CV176 = 1** by POM. The loco starts immediately: it ramps to full power over 0.6 s, holds for 1 s while
   it settles and measures, then stops on its own.
5. On success the decoder stores the new CV5 and CV6 and reboots. The loco is ready to use with the new speed
   table.

To abort a run, change the throttle (any speed or direction change) or press emergency stop. The motor stops, CV5
and CV6 are left unchanged, and the decoder then follows the throttle as usual.

The decoder can't report the outcome on the main track. To check it, read CV176 on the programming track:

| CV176 | Meaning | What to do |
|---|---|---|
| 0 | No calibration has been run (or the status was cleared) | |
| 2 | Done: CV5 and CV6 were updated | |
| 3 | Aborted, or refused because the loco wasn't stopped | Stop the loco and try again |
| 4 | No motion: almost no BEMF at the end of the ramp | Check the motor connection and that the wheels are free |
| 5 | Unstable reading: the BEMF changed by more than 3% while measuring | Clean the track and wheels; the loco may also have still been accelerating |
| 6 | Out of range: the measured BEMF is too low to give a usable CV5 | Check the BEMF measurement settings (CV61–CV64) |

Writing CV176 = 0 clears the status. Other values are rejected.

#### Configuration variables

| CV | Name | Default | Set by | Description |
|---|---|---|---|---|
| 176 | Motor calibration | 0 | User / decoder | Write 1 by POM to start a run. Afterwards holds the status from the table above. |
| 177–178 | Calibrated EMF max | 0 | Decoder | The BEMF measured at 100% PWM in ADC counts, after subtracting the ADC offset (16 bit, CV177 is the high byte). Read-back only; useful for comparing locos or track voltages. |
| 179 | Calibration headroom | 235 | User | How close to the measured maximum the top speed step is placed. See below. |
| 5 | Vhigh | 100 | Decoder | BEMF target at speed step 126, in units of 16 ADC counts. Rewritten by a successful run. |
| 6 | Vmid | 63 | Decoder | BEMF target at speed step 63. Rescaled by a successful run. |

**CV179, calibration headroom.** The new CV5 is `measured EMF max × CV179 / 255 / 16`, so CV179 sets the top speed
step as a fraction of the speed the motor reaches at 100% PWM. The default of 235 is about 92%.

- The speed controller needs spare PWM to hold a speed when the load goes up (a grade, a heavy train). With 92%,
  it has roughly 8% in reserve at step 126.
- A **lower** value gives more reserve, so top speed is held better under load, but the top speed is lower.
- A **higher** value gives a higher top speed, but the highest speed steps may run out of PWM under load and stop
  responding. 255 targets the full measured maximum and leaves no reserve.
- The calibration is done with the loco running light. If you calibrate on rollers and then pull heavy trains,
  consider a lower value.

CV179 only takes effect when a calibration runs, so re-run the calibration after changing it.

**CV6, Vmid.** The run keeps the shape of your speed curve: CV6 is scaled by the same factor as CV5, so the
CV6/CV5 ratio stays the same (a linear curve stays linear). CV6 is never set higher than CV5. To change the curve,
write CV6 after calibrating.

**Settings that affect the result.** The run uses the same BEMF measurement as normal running, so calibrate again
after changing any of these, and whenever the track voltage changes:

- Motor PWM frequency (CV9) and the BEMF measurement settings: sample count (CV61), measurement delay (CV62) and
  the low / high outlier cutoffs (CV63, CV64).
- The ADC offset (CV172). It is measured automatically on first start; writing CV7 = 7 measures it again.

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
