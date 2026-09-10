# Mobile Vision Inspection — Full Change Report

This covers everything across our conversation: the ALT light controller
protocol bug, the multi-file app's connection bugs, the UI rework, and the
final modular restructure. Three deliverables came out of it:

1. `alt_light_controller.py` — standalone light-controller GUI, protocol fixed
2. `vision_inspection_app.py` — full inspection app, single file, protocol + topology + UI fixed
3. `mobile_inspection_app_modular.zip` — same app, split into a proper modular package

---

## 1. The original bug: wrong ALT-E4RS wire protocol

### What was wrong
Your light controller GUI was building frames like this:

```
EF EF 00 [ch1] [ch2] [ch3] [ch4] [checksum] EE EE
```
with a checksum computed as `XOR of channels 1-3, XOR'd with (channel4 + 1)`.

This was never confirmed against your actual ALT-E4RS-12V hardware — it was
guessed from a partial/incomplete C# reference (`AltLight.cs`), and the code
even had a comment on it: `"Unverified for this exact unit"`. Because the
header, footer, and checksum were all wrong, the device never recognized the
command — no error, just no light.

### Root cause
The controller's real firmware expects a completely different frame,
reverse-engineered from `Viskit.Components.Light.AltLightERS
.ChangeLightValue4Ch` and confirmed working over RS-232 at 9600 8N1:

```
4C 15 [ch1] [ch2] [ch3] [ch4] [checksum] 0D 0A
```
where `checksum = byte[1] ^ byte[2] ^ byte[3] ^ byte[4] ^ byte[5]` (XOR of
the header's second byte and all four channel values), and the frame ends
with a standard CR LF terminator instead of a proprietary footer.

### Fix
Replaced the frame-building logic everywhere with this exact format.
Verified independently with test vectors, e.g.:
```
build_frame([0,0,0,0])       -> 4C 15 00 00 00 00 15 0D 0A
build_frame([255,128,0,64])  -> 4C 15 FF 80 00 40 2A 0D 0A
```

---

## 2. `alt_light_controller.py` — standalone controller GUI

Your original `AltLightControllerGUI` (TCP/UDP/Serial transports, byte
monitor, slider throttling) kept as-is structurally. Changed:

- **`build_frame`**: replaced with the verified `4C 15 ... 0D 0A` format above.
- **Removed 8-channel mode**: your unit's label reads ALT-E4RS — a fixed
  4-channel model. The verified protocol only covers 4 channels; sending an
  8-channel packet to 4-channel hardware was always going to do nothing at
  best.
- **Default baud**: changed from 19200 → 9600, since 9600 is what was
  actually confirmed to work.
- **Byte Monitor default hex**: updated to match the new frame
  (`4C 15 00 00 00 00 15 0D 0A`) so "Send Raw Hex" still demos a valid frame.
- Kept: TCP/UDP transports, the byte monitor, the 100ms send-throttling
  logic (unchanged — that logic was already correct).

---

## 3. `mobile_inspection_app_modular.zip` v5 → fixed — bugs found

You uploaded a bigger app (`app.py` + `lighting.py` + `alt_ers_protocol.py`
+ `camera.py` + `capture_manager.py` + `config.py` + `detection.py`). Three
separate, real bugs were found in it:

### Bug A — `could not open PORT COM1: PermissionError(13, 'Access is denied')`

**Root cause:** In `_build_light_ports()`, both the Left and Right
controller port dropdowns defaulted to `ports[0]` whenever nothing had been
picked yet:
```python
for var, combo in ((self.left_port_var, self.left_port_combo), (self.right_port_var, self.right_port_combo)):
    if var.get() not in ports:
        var.set(ports[0] if ports else "SIMULATOR")
```
Both variables get set to the **same** port. When you hit Connect for
`LIGHT_DUAL_4`, it opened that port for the left controller (succeeded),
then tried to open the *same* port again for the right controller.
Windows refuses a second open of a COM port that's already held open —
that's exactly `PermissionError(13, 'Access is denied')`.

**Fix:** Left now defaults to `ports[0]`, Right now defaults to `ports[1]`
(a different port) whenever more than one port is available. On top of
that, `Connect` now checks up front and refuses with a clear message if
Left and Right are ever set to the same real port:
> "Left and Right are set to the same port. Two 4-channel controllers need
> two DIFFERENT COM ports."

### Bug B — `LIGHT_SINGLE_8` does nothing, no error at all

**Root cause:** `build_8channel_frame()` in `alt_ers_protocol.py` is an
**inferred, unverified extension** of the confirmed 4-channel frame — it
was never actually tested against real 8-channel hardware. The write
succeeds (no exception is raised), the device just silently doesn't
recognize the frame, so nothing lights up and nothing errors.

**Fix:** Kept the 8-channel path (in case you do get 8-channel hardware
later) but the UI now shows a persistent orange warning banner whenever
`LIGHT_SINGLE_8` is selected: *"⚠ 8-channel frame format is NOT confirmed
against real hardware."* It's clearly a different trust level from the
4-channel path now, instead of silently pretending to work.

### Bug C — no topology actually matched your hardware

**Root cause:** You described your actual setup as *one* ALT-E4RS
4-channel controller driving *one* RGB light on channels 1–3 (channel 4
unused). The app only offered:
- `LIGHT_DUAL_4` — needs **two** controllers / two ports (not what you have)
- `LIGHT_SINGLE_8` — needs **8-channel hardware** (also not what you have,
  and unverified besides)

Neither matched, which is *why* you were forced into `LIGHT_DUAL_4` with
both ports pointing at your one real controller — which is what triggered
Bug A in the first place.

**Fix:** Added a new topology, **`LIGHT_SINGLE_4`** — one controller, one
RGB light on CH1–3 (R, G, B), CH4 left at 0. This is now the **default**
topology on launch. It uses the confirmed 4-channel frame directly, one
port, no possibility of a port collision.

---

## 4. UI changes (both the standalone controller and the full app)

- Modern flat palette: indigo accent (`#4f46e5`), soft off-white background,
  clean white cards, consistent spacing/typography (Segoe UI throughout).
- **Colored connection status**: 🔴 "● not connected" / 🟢 "● connected"
  next to the Connect button, instead of only a popup.
- **Per-recipe "Test" button**: every lighting recipe row (WARM / BLUE /
  SKY_GREEN / NEUTRAL, and their Left/Right equivalents in dual mode) now
  has its own **Test** button that drives exactly that RGB state
  immediately — you can eyeball a light without touching the Capture tab.
  "READY (white)" and "ALL OFF" remain as global controls.
- Friendlier connection-error text: instead of a bare Windows exception,
  `AltErs*Controller.connect()` now wraps failures with the likely causes
  (port already open elsewhere, wrong COM number, needs admin rights).
- Camera, Capture, and Detection card logic unchanged from v5 — only their
  visual styling changed.

---

## 5. Why it went from "single file" to "modular package"

The single-file version (`vision_inspection_app.py`) was what you'd asked
for at that point. You then asked for real modularity, so the same app was
restructured — same behaviour, same fixes above, but split by
responsibility and written against interfaces instead of concrete classes,
so it follows SOLID:

- **Single Responsibility** — one file per concern: wire-protocol bytes
  (`lighting/protocol.py`) is separate from serial I/O
  (`lighting/controllers.py`), which is separate from topology logic
  (`lighting/engines.py`), which is separate from hardware *wiring*
  (`lighting/factory.py`). Same split for camera / detection / capture / UI.
- **Dependency Inversion** — `Single4LightEngine`, `Dual4LightEngine`, etc.
  are written against the `ChannelController` interface
  (`interfaces.py`), never against `AltErs4Controller` directly. A
  `SimulatedController` is interchangeable with a real one everywhere.
- **Open/Closed** — `lighting/factory.py` is the *only* place that knows
  "topology X needs these ports/controllers." Adding a new topology later
  means adding one branch there — the UI and engines never change. Same
  pattern for detectors: a new one is a new class registered with
  `DetectionRegistry`, nothing else changes.
- **Mediator (UI)** — `CameraPanel`, `LightingPanel`, `CapturePanel`,
  `DetectionPanel` never reference each other. `ui/app.py`'s `Application`
  class is the only thing that knows all of them exist, and wires them
  together with plain callbacks (e.g. it's told whenever `LightingPanel`
  reconnects, and forwards the new engine to `CaptureManager`).

Full package layout and rationale is also written into `README.md` inside
the zip.

---

## 6. Full description of what the app does now

**Camera card** — enable Camera 1 / Camera 2 (Basler, via `pypylon`),
choose the live view (Stitched / Camera 1 / Camera 2), Start/Stop.

**Lighting card**
- Pick a topology: `LIGHT_SINGLE_4` (default — your hardware),
  `LIGHT_DUAL_4`, or `LIGHT_SINGLE_8` (flagged unverified).
- Pick COM port(s) (or `SIMULATOR` to test with no hardware attached — logs
  what *would* be sent instead of opening a real port).
- Connect / READY (drives white on all channels) / ALL OFF.
- Up to 4 named RGB recipes (defaults: WARM, BLUE, SKY_GREEN, NEUTRAL),
  each with its own Test button, each individually enabled/disabled for
  capture via its checkbox.

**Capture card** — runs through every *enabled* recipe: turn light(s) on →
wait `settle_ms` → grab a frame from each enabled camera → turn light(s)
off → save. Saves per-camera PNGs and/or a stitched PNG (configurable),
plus a `manifest.txt` per capture set recording exactly what was
saved and with what RGB values, into `captures/<model>/set<N>/`.

**Detection card** — run "RAW Laplacian" or "Canny" edge detection on the
current live frame, or load a custom `.pt` (YOLO/Ultralytics) model to run
instead. Purely for a quick visual sanity check right now — the
`ModelPT` class is a stub ready for a real defect-decision pipeline later.

**System Log card** — every connect/disconnect, every TX frame in hex,
every capture step, and every error gets written here, so if something
still doesn't behave you (or I, next time) can see exactly what byte went
out and when.
