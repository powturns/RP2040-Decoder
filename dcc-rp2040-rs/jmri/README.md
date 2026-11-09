# JMRI DecoderPro definition

`Public_Domain_RP2040_Decoder.xml` lets JMRI DecoderPro read and write the CVs of the Rust
firmware (`dcc-rp2040-rs`) with a friendly UI.

It appears as **Public-domain and DIY → RP2040-Decoder → RP2040-Decoder (Rust firmware)** and is
auto-detected from CV8 = 13 and CV7 = 129.

## Install

1. In DecoderPro, open **Help → Locations** and note the *User Files Location*.
2. Copy `Public_Domain_RP2040_Decoder.xml` into a `decoders` folder inside that location
   (create the folder if it does not exist).
3. In the DecoderPro main window choose **Debug → Recreate Decoder Index**
   (older versions: **Actions → Recreate Decoder Index**) and restart DecoderPro.

Programming modes: Direct (byte and bit) on the programming track, and Ops mode writes on the
main. Paged/register modes are not supported by the firmware.

## What is exposed

| Pane | CVs |
|------|-----|
| Standard Basic / Motor panes | 1, 17/18, 29 (address format), 2–6, 7, 8 |
| RP2040 Motor | speed curve, momentum, 9 (motor PWM Hz), 174 (motor PWM divider), 175 (speed step period), 176–179 (motor calibration) |
| RP2040 PID | 47–51, 54–59 (16-bit Kp values), 60 |
| RP2040 Back-EMF | 61–64, 172 (ADC offset, 255 = re-measure at boot) |
| RP2040 Output PWM | 112–115 PWM enable bits for the wired outputs, slice config for slices 0, 1, 2, 4, 5 |
| Function Map (standard) | F0–F28 forward/reverse → GPIO0–5, AUX0–3 (CV257+) |
| RP2040 F29-F31 Map | F29–F31 forward/reverse (JMRI's standard grid stops at F28) |

Values are shown in the units the CV is stored in, unless the tooltip says otherwise. For example,
Ki is entered as `CV50 = Ki × 10`. CV9 and the slice dividers are shown in real units
(Hz and divider), and JMRI converts them back when writing.

Most motor, PID and output settings are read only at power-up, so power-cycle the decoder after
writing them.

### Motor calibration

The **RP2040 Motor** pane has the calibration run controls (see the firmware README for the full
procedure). Open the loco in **ops mode** (Programming on Main), set *Calibration control / status*
to **Start calibration run** and write that one value: the loco immediately runs forward at full
speed for about 1.6 s, rewrites CV5/CV6 and reboots. Read the sheet on the programming track
afterwards to see the status, the measured EMF max and the new CV5/CV6.

The firmware only accepts 0 (Idle) and 1 (Start) for CV176, and never stores 1. Two consequences:

- After a run, DecoderPro still shows **Start calibration run** until you read the value back or
  set it to Idle. Do that before using *Write all sheets* in ops mode, or the loco will run again.
- *Write all sheets* on the programming track reports an error on CV176 when it holds a status
  (2–6), or Start. Set it to Idle first.

Writing 8 to CV8 (**Reset → Reset All CVs** in DecoderPro) restores factory defaults.

Not exposed because the current firmware ignores them: CV11, CV19 (consist), CV29 bits other than
the address format, CV52/53, CV65, CV173.

## Regenerating

The XML is generated (the function map alone has 640 variables). Edit `generate.py`, then run:

```sh
python3 generate.py
```

Keep `generate.py` in sync with `dcc/src/cv/mod.rs` / `dcc/src/cv/store.rs`, and bump
`DEF_VERSION`/`LAST_UPDATED` when the definition changes. If the firmware's CV7 version
changes, update `VERSION_ID` so auto-detection keeps working.
