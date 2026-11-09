#!/usr/bin/env python3
"""Generate the JMRI DecoderPro definition for the RP2040-Decoder (Rust firmware).

The function map alone is 32 functions x 2 directions x 10 outputs = 640 variables, so the
XML is generated rather than hand-written. Keep the CV numbers and defaults here in sync with
`dcc/src/cv/mod.rs` and `dcc/src/cv/store.rs`.

Usage:
    python3 generate.py            # writes Public_Domain_RP2040_Decoder.xml next to this script
"""

from __future__ import annotations

from pathlib import Path
from xml.sax.saxutils import escape, quoteattr

OUT_FILE = Path(__file__).with_name("Public_Domain_RP2040_Decoder.xml")

FAMILY = "RP2040-Decoder"
MODEL = "RP2040-Decoder (Rust firmware)"
MFG = "Public-domain and DIY"
VERSION_ID = 129  # CV7 default in dcc/src/cv/mod.rs
DEF_VERSION = 2
LAST_UPDATED = "20261110"  # bump with DEF_VERSION when the definition changes

# Wired function outputs, in the order they appear as columns in the function map.
# (label, gpio number). AUX0-3 are GPIO24-27.
OUTPUTS = [
    ("GPIO|0", 0),
    ("GPIO|1", 1),
    ("GPIO|2", 2),
    ("GPIO|3", 3),
    ("GPIO|4", 4),
    ("GPIO|5", 5),
    ("AUX|0", 24),
    ("AUX|1", 25),
    ("AUX|2", 26),
    ("AUX|3", 27),
]

NUM_FUNCTIONS = 32  # F0..F31
STD_FNMAP_MAX = 28  # JMRI's standard Function Map pane only shows up to F28
FNMAP_ROWS = (STD_FNMAP_MAX + 1) * 2
FUNCTION_MAP_BASE = 257  # CV257 = F0 forward, byte 3 (GPIO 24-31)
FUNCTION_MAP_STRIDE = 8
FUNCTION_MAP_DIR_OFFSET = 4

PWM_ENABLE_BASE = 112  # CV112..115, big endian u32 of GPIO bits
PWM_SLICE_BASE = 116
PWM_SLICE_STRIDE = 7

# PWM slices that drive wired outputs: slice -> (channel A output label, channel B output label)
PWM_SLICES = {
    0: ("GPIO0", "GPIO1"),
    1: ("GPIO2", "GPIO3"),
    2: ("GPIO4", "GPIO5"),
    4: ("AUX0", "AUX1"),
    5: ("AUX2", "AUX3"),
}


def mask_for_bit(bit: int) -> str:
    """JMRI mask string selecting a single bit (rightmost char is bit 0)."""
    return "".join("V" if (7 - i) == bit else "X" for i in range(8))


def u32_cv_and_bit(base_cv: int, gpio: int) -> tuple[int, int]:
    """Locate a GPIO bit inside a big-endian u32 spread over CV base..base+3."""
    byte_index_from_lsb = gpio // 8
    return base_cv + (3 - byte_index_from_lsb), gpio % 8


class Writer:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def __call__(self, indent: int, text: str = "") -> None:
        self.lines.append(("  " * indent + text) if text else "")

    def text(self) -> str:
        return "\n".join(self.lines) + "\n"


def variable(
    w: Writer,
    *,
    cv: int,
    item: str,
    label: str,
    value: str,
    tooltip: str | None = None,
    default: int | str | None = None,
    mask: str | None = None,
    read_only: bool = False,
    indent: int = 3,
) -> None:
    attrs = [f'CV="{cv}"', f"item={quoteattr(item)}"]
    if mask:
        attrs.append(f'mask="{mask}"')
    if default is not None:
        attrs.append(f'default="{default}"')
    if read_only:
        attrs.append('readOnly="yes"')
    w(indent, f"<variable {' '.join(attrs)}>")
    w(indent + 1, value)
    w(indent + 1, f"<label>{escape(label)}</label>")
    if tooltip:
        w(indent + 1, f"<tooltip>{escape(tooltip)}</tooltip>")
    w(indent, "</variable>")


def dec(min_: int = 0, max_: int = 255, factor: int | None = None, offset: int | None = None) -> str:
    extra = ""
    if factor is not None:
        extra += f' factor="{factor}"'
    if offset is not None:
        extra += f' offset="{offset}"'
    return f'<decVal min="{min_}" max="{max_}"{extra}/>'


def split16(low_cv_minus_one: int) -> tuple[int, str]:
    """A big-endian u16 stored at CV n (high) and n+1 (low)."""
    return low_cv_minus_one + 1, f'<splitVal highCV="{low_cv_minus_one}" min="0" max="65535"/>'


def enum(choices: list[str]) -> str:
    """An enum whose choices map to 0, 1, 2... in order."""
    return "<enumVal>" + "".join(f"<enumChoice choice={quoteattr(c)}/>" for c in choices) + "</enumVal>"


# CV176 values, in order from 0. Keep in sync with motor/src/speed/calibration.rs.
CALIBRATION_STATES = [
    "Idle",
    "Start calibration run",
    "Done - CV5/CV6 updated",
    "Aborted / loco not stopped",
    "Failed - no motion",
    "Failed - unstable reading",
    "Failed - out of range",
]

NO_YES = '<xi:include href="http://jmri.org/xml/decoders/parts/enum-NoYes.xml"/>'


def write_variables(w: Writer) -> None:
    w(2, "<variables>")
    w(3, "<!-- CV1, CV17/18, CV29 bit 5 -->")
    w(3, '<xi:include href="http://jmri.org/xml/decoders/nmra/shortAndLongAddress.xml"/>')

    w(3, "<!-- Speed curve: CV2 is in raw back-EMF ADC counts, CV5/CV6 are multiplied by 16 -->")
    variable(w, cv=2, item="Vstart", label="Start voltage (Vstart)", value=dec(),
             default=8,
             tooltip="Back-EMF target at speed step 1, in raw 12-bit ADC counts (not multiplied by 16).")
    variable(w, cv=3, item="Accel", label="Acceleration rate", value=dec(), default=0,
             tooltip="Time per speed step while accelerating = CV3 x CV175 ms. 0 = no momentum.")
    variable(w, cv=4, item="Decel", label="Deceleration rate", value=dec(), default=0,
             tooltip="Time per speed step while decelerating = CV4 x CV175 ms. 0 = no momentum.")
    variable(w, cv=5, item="Vhigh", label="Max voltage (Vhigh)", value=dec(), default=100,
             tooltip="Back-EMF target at the top speed step = CV5 x 16 ADC counts (255 = full 12-bit range). "
                     "Rewritten by a successful motor calibration run (CV176).")
    variable(w, cv=6, item="Vmid", label="Mid voltage (Vmid)", value=dec(), default=63,
             tooltip="Back-EMF target at speed step 63 = CV6 x 16 ADC counts. "
                     "Rescaled by a successful motor calibration run, keeping the CV6/CV5 ratio.")

    variable(w, cv=7, item="Decoder Version", label="Firmware version", value="<decVal/>",
             default=VERSION_ID, read_only=True)
    variable(w, cv=8, item="Manufacturer", label="Manufacturer ID", value="<decVal/>",
             default=13, read_only=True,
             tooltip="13 = Public-domain and DIY. Writing 8 to CV8 resets all CVs to factory defaults.")

    w(3, "<!-- Motor drive -->")
    variable(w, cv=9, item="Motor PWM frequency", label="Motor PWM frequency (Hz)",
             value=dec(10000, 35500, factor=100, offset=10000), default=25000,
             tooltip="Stored as (Hz - 10000) / 100. Takes effect after a power cycle.")
    variable(w, cv=174, item="Motor PWM clock divider", label="Motor PWM clock divider",
             value=dec(0, 255), default=1,
             tooltip="Additional motor PWM clock divider. 0 is treated as 1. Takes effect after a power cycle.")
    variable(w, cv=175, item="Speed step period", label="Speed step period (ms)",
             value=dec(0, 255), default=7,
             tooltip="Base period of the acceleration/deceleration ramp; multiplies CV3 and CV4.")

    w(3, "<!-- Motor calibration run -->")
    variable(w, cv=176, item="Motor calibration", label="Calibration control / status",
             value=enum(CALIBRATION_STATES), default=0,
             tooltip="Ops mode (on the main) only: writing 'Start calibration run' makes the loco run FORWARD "
                     "at full speed for about 1.6 s, then it rewrites CV5/CV6 and reboots. The loco must be "
                     "stopped; any throttle change aborts. Read on the programming track to see the result. "
                     "Only 'Idle' (clears the status) and 'Start calibration run' can be written.")
    cv, val = split16(177)
    variable(w, cv=cv, item="Motor calibration EMF max", label="Calibrated EMF max (ADC counts)",
             value=val, default=0, read_only=True,
             tooltip="Back-EMF measured at 100% PWM by the last calibration run, after subtracting the ADC "
                     "offset. Stored big-endian in CV177 (high) and CV178 (low).")
    variable(w, cv=179, item="Motor calibration headroom", label="Calibration headroom (CV/255)",
             value=dec(), default=235,
             tooltip="A calibration run sets CV5 = EMF max x CV179 / 255 / 16. Default 235 = 92%: the top "
                     "speed step targets 92% of the speed reached at full power, leaving the controller "
                     "reserve to hold speed under load. Lower = more reserve but lower top speed; 255 = no "
                     "reserve. Only used when a calibration runs.")

    w(3, "<!-- PID speed controller -->")
    variable(w, cv=47, item="PID feed-forward", label="Startup seed k_ff (CV/255)",
             value=dec(), default=230,
             tooltip="Fraction of the startup drive level used to seed the PID when leaving startup mode = CV47 / 255.")
    variable(w, cv=48, item="PID filter time constant", label="Derivative filter time constant (ms)",
             value=dec(), default=10)
    variable(w, cv=49, item="PID sample time", label="PID sample time (ms)",
             value=dec(1, 255), default=5)
    variable(w, cv=50, item="PID Ki", label="Ki (x10)", value=dec(), default=25,
             tooltip="Integral gain = CV50 / 10. Default 25 = 2.5.")
    variable(w, cv=51, item="PID Kd", label="Kd (x10000)", value=dec(), default=50,
             tooltip="Derivative gain = CV51 / 10000. Default 50 = 0.005.")
    for (hi, item, label, default, tip) in [
        (54, "PID Kp x0", "Kp at standstill (x100)", 2000, "Kp at the lowest setpoint = value / 100. Default 2000 = 20."),
        (56, "PID Kp x1", "Kp at gain-range end (x100)", 250, "Kp at the point set by CV60 = value / 100. Default 250 = 2.5."),
        (58, "PID Kp x2", "Kp at max speed (x100)", 150, "Kp at the highest setpoint = value / 100. Default 150 = 1.5."),
    ]:
        cv, val = split16(hi)
        variable(w, cv=cv, item=item, label=label, value=val, default=default,
                 tooltip=f"{tip} Stored big-endian in CV{hi} (high) and CV{hi + 1} (low).")
    variable(w, cv=60, item="PID Kp gain range end", label="Kp gain-range shift (CV/255)",
             value=dec(), default=13,
             tooltip="Position of the Kp x1 point as a fraction of the setpoint range = CV60 / 255.")

    w(3, "<!-- Back-EMF measurement -->")
    variable(w, cv=61, item="EMF samples", label="Samples per measurement", value=dec(1, 255), default=100)
    variable(w, cv=62, item="EMF delay", label="Delay before measuring (us)", value=dec(), default=100,
             tooltip="Time between cutting motor power and sampling back-EMF.")
    variable(w, cv=63, item="EMF low cutoff", label="Discard lowest samples", value=dec(), default=15,
             tooltip="Number of smallest samples discarded as outliers. CV63 + CV64 must be less than CV61.")
    variable(w, cv=64, item="EMF high cutoff", label="Discard highest samples", value=dec(), default=15,
             tooltip="Number of largest samples discarded as outliers. CV63 + CV64 must be less than CV61.")
    variable(w, cv=172, item="EMF ADC offset", label="ADC offset", value=dec(), default=255,
             tooltip="Measured automatically at boot when 255. Write 255 to force a re-measurement on next power-up.")

    w(3, "<!-- Function output PWM: enable bits (CV112..115 big-endian u32 of GPIO bits) -->")
    for label, gpio in OUTPUTS:
        name = label.replace("|", "")
        cv, bit = u32_cv_and_bit(PWM_ENABLE_BASE, gpio)
        variable(w, cv=cv, mask=mask_for_bit(bit), item=f"PWM enable {name}",
                 label=f"{name} uses PWM", value=NO_YES, default=0,
                 tooltip=f"GPIO{gpio}: CV{cv} bit {bit}. When off the output is plain on/off.")

    w(3, "<!-- Function output PWM: per-slice configuration -->")
    for slice_no, (a, b) in PWM_SLICES.items():
        base = PWM_SLICE_BASE + slice_no * PWM_SLICE_STRIDE
        cv, val = split16(base)
        variable(w, cv=cv, item=f"PWM slice {slice_no} wrap", label=f"Slice {slice_no} ({a}/{b}) wrap",
                 value=val, default=0,
                 tooltip=f"Counter top value. PWM frequency = 125 MHz / (divider x (wrap + 1)). CV{base}/CV{base + 1}.")
        variable(w, cv=base + 2, item=f"PWM slice {slice_no} divider", label=f"Slice {slice_no} clock divider",
                 value=dec(1, 256, offset=1), default=1,
                 tooltip=f"Stored in CV{base + 2} as divider - 1.")
        cv, val = split16(base + 3)
        variable(w, cv=cv, item=f"PWM slice {slice_no} level A", label=f"{a} level",
                 value=val, default=0,
                 tooltip=f"Duty cycle = level / (wrap + 1) when {a} is on. CV{base + 3}/CV{base + 4}.")
        cv, val = split16(base + 5)
        variable(w, cv=cv, item=f"PWM slice {slice_no} level B", label=f"{b} level",
                 value=val, default=0,
                 tooltip=f"Duty cycle = level / (wrap + 1) when {b} is on. CV{base + 5}/CV{base + 6}.")

    w(3, "<!-- Function map: F0-F31, forward/reverse, each a big-endian u32 GPIO mask -->")
    for fn in range(NUM_FUNCTIONS):
        for dir_tag, dir_offset in (("f", 0), ("r", FUNCTION_MAP_DIR_OFFSET)):
            base = FUNCTION_MAP_BASE + fn * FUNCTION_MAP_STRIDE + dir_offset
            for out_no, (label, gpio) in enumerate(OUTPUTS, start=1):
                cv, bit = u32_cv_and_bit(base, gpio)
                item = f"F{fn}({dir_tag}) controls output {out_no}"
                w(3, f'<variable item="{item}" CV="{cv}" mask="{mask_for_bit(bit)}" '
                     f'minFn="{fn}" minOut="{out_no}" default="0">')
                w(4, "<enumVal><enumChoice choice=\"No\"/><enumChoice choice=\"Yes\"/></enumVal>")
                w(4, f"<label>{item}</label>")
                w(3, "</variable>")
    w(2, "</variables>")


def display(w: Writer, indent: int, item: str, fmt: str | None = None) -> None:
    f = f' format="{fmt}"' if fmt else ""
    w(indent, f'<display item={quoteattr(item)}{f}/>')


def heading(w: Writer, indent: int, text: str) -> None:
    w(indent, f"<label><text>&lt;html&gt;&lt;b&gt;{escape(text)}&lt;/b&gt;&lt;/html&gt;</text></label>")


def write_panes(w: Writer) -> None:
    # Motor
    w(1, "<pane>")
    w(2, "<name>RP2040 Motor</name>")
    w(2, "<column>")
    heading(w, 3, "Speed curve")
    for item in ("Vstart", "Vmid", "Vhigh"):
        display(w, 3, item)
    w(3, "<separator/>")
    heading(w, 3, "Momentum")
    for item in ("Accel", "Decel", "Speed step period"):
        display(w, 3, item)
    w(3, "<separator/>")
    heading(w, 3, "Motor drive (applied after power cycle)")
    display(w, 3, "Motor PWM frequency")
    display(w, 3, "Motor PWM clock divider")
    w(2, "</column>")
    w(2, "<column>")
    heading(w, 3, "Calibration run (start in ops mode, on the main)")
    for item in ("Motor calibration headroom", "Motor calibration", "Motor calibration EMF max"):
        display(w, 3, item)
    w(2, "</column>")
    w(1, "</pane>")

    # PID
    w(1, "<pane>")
    w(2, "<name>RP2040 PID</name>")
    w(2, "<column>")
    heading(w, 3, "Proportional gain curve")
    for item in ("PID Kp x0", "PID Kp x1", "PID Kp x2", "PID Kp gain range end"):
        display(w, 3, item)
    w(3, "<separator/>")
    heading(w, 3, "Integral / derivative")
    for item in ("PID Ki", "PID Kd", "PID filter time constant", "PID sample time"):
        display(w, 3, item)
    w(3, "<separator/>")
    heading(w, 3, "Startup")
    display(w, 3, "PID feed-forward")
    w(2, "</column>")
    w(1, "</pane>")

    # Back-EMF
    w(1, "<pane>")
    w(2, "<name>RP2040 Back-EMF</name>")
    w(2, "<column>")
    for item in ("EMF samples", "EMF delay", "EMF low cutoff", "EMF high cutoff", "EMF ADC offset"):
        display(w, 3, item)
    w(2, "</column>")
    w(1, "</pane>")

    # Output PWM
    w(1, "<pane>")
    w(2, "<name>RP2040 Output PWM</name>")
    w(2, "<column>")
    heading(w, 3, "PWM enable (applied after power cycle)")
    for label, _ in OUTPUTS:
        display(w, 3, f"PWM enable {label.replace('|', '')}", "checkbox")
    w(2, "</column>")
    w(2, "<column>")
    for slice_no, (a, b) in PWM_SLICES.items():
        heading(w, 3, f"Slice {slice_no}: {a} / {b}")
        for suffix in ("wrap", "divider", "level A", "level B"):
            display(w, 3, f"PWM slice {slice_no} {suffix}")
        w(3, "<separator/>")
    w(2, "</column>")
    w(1, "</pane>")

    # JMRI's standard Function Map pane stops at F28 (FnMapPanel.highestFn is hard-coded),
    # so F29-F31 get their own pane.
    w(1, "<pane>")
    w(2, f"<name>RP2040 F{STD_FNMAP_MAX + 1}-F{NUM_FUNCTIONS - 1} Map</name>")
    for fn in range(STD_FNMAP_MAX + 1, NUM_FUNCTIONS):
        for dir_tag, dir_name in (("f", "forward"), ("r", "reverse")):
            w(2, "<column>")
            heading(w, 3, f"F{fn} {dir_name}")
            for out_no, (label, _) in enumerate(OUTPUTS, start=1):
                w(3, f'<display item="F{fn}({dir_tag}) controls output {out_no}" format="checkbox">')
                w(4, f"<label>{label.replace('|', '')}</label>")
                w(3, "</display>")
            w(2, "</column>")
    w(1, "</pane>")


def main() -> None:
    w = Writer()
    w(0, '<?xml version="1.0" encoding="utf-8"?>')
    w(0, '<?xml-stylesheet type="text/xsl" href="../XSLT/decoder.xsl"?>')
    w(0, "<!-- GENERATED by dcc-rp2040-rs/jmri/generate.py - edit the generator, not this file. -->")
    w(0, '<decoder-config xmlns:xi="http://www.w3.org/2001/XInclude" '
         'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
         'xsi:noNamespaceSchemaLocation="http://jmri.org/xml/schema/decoder-4-15-2.xsd" '
         'showEmptyPanes="no">')
    w(1, f'<version author="RP2040-Decoder project" version="{DEF_VERSION}" '
         f'lastUpdated="{LAST_UPDATED}"/>')
    w(1, "<decoder>")
    w(2, f'<family name="{FAMILY}" mfg="{MFG}" lowVersionID="{VERSION_ID}" highVersionID="{VERSION_ID}" '
         'comment="Open-source RP2040 based DCC decoder, Rust firmware (dcc-rp2040-rs)">')
    # numFns caps the number of *rows* in JMRI's Function Map grid (one per F<n>(f)/F<n>(r)),
    # not the function count, so it must cover F0..F28 x 2 directions.
    w(3, f'<model model="{MODEL}" numOuts="{len(OUTPUTS)}" numFns="{FNMAP_ROWS}" '
         f'maxFnNum="{NUM_FUNCTIONS - 1}" formFactor="HO" connector="other">')
    for out_no, (label, gpio) in enumerate(OUTPUTS, start=1):
        w(4, f'<output name="{out_no}" label="{label}" connection="solder"/>')
    w(4, '<size length="25" width="20" units="mm"/>')
    w(3, "</model>")
    w(2, "</family>")
    w(2, '<programming direct="yes" paged="no" register="no" ops="yes"/>')
    write_variables(w)
    w(2, "<resets>")
    w(3, '<factReset label="Reset All CVs" CV="8" default="8">')
    w(4, "<label>Reset All CVs</label>")
    w(3, "</factReset>")
    w(2, "</resets>")
    w(1, "</decoder>")
    write_panes(w)
    w(0, "</decoder-config>")
    OUT_FILE.write_text(w.text(), encoding="utf-8")
    print(f"wrote {OUT_FILE}")


if __name__ == "__main__":
    main()
