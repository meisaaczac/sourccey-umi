# Step-by-Step Assembly Guide

Build your **Sourccey UMI** handheld gripper from start to finish. Every mechanical interface matches the real Vulcan Robotics Sourccey gripper so the recorded opening angle (`q`) and wrist-camera viewpoint transfer directly to the robot.

| Build Time | Difficulty | Printed Mass | Special Tools |
|---|---|---|---|
| **~45–60 min** *(excluding printing)* | Intermediate | ~187 g (PETG) | Soldering iron (for M3 heat-set inserts) |

![Exploded assembly overview](img/00_exploded_overview.png)

---

## Before You Start

### Tools Required
- [ ] Soldering iron with conical or heat-set insert tip (~220 °C for PETG)
- [ ] Hex keys (2.0 mm / 2.5 mm for M3 socket/pan-head screws)
- [ ] Needle-nose pliers (for hooking the extension spring)
- [ ] Scissors (for trimming the 15 mm velcro finger strap)
- [ ] *Optional:* Flush cutters & deburring tool to clean up print brims/supports

### Parts & Fasteners Checklist
Stage all parts on your workbench before starting (full specs in [`BOM.md`](../BOM.md)):

| Qty | Component | Used In |
|---|---|---|
| 4 | Printed parts: **Base**, **Finger**, **Pistol Grip**, **Trigger Lever** | Steps 1–5 |
| 5 | M3 heat-set inserts (for Ø4.2 mm holes) | Step 2 |
| 5 | M3 × 20 mm screws (pan head) | Step 3 |
| 4 | M3 × 16 mm screws (pan head) | Step 4 |
| 1 | Feetech STS3215 7.4 V servo + horn (torque off / read-only) | Step 5 |
| 1 | M3 × 70 mm screw + M3 nut *(or 3 mm smooth rod)* | Step 6 |
| 1 | Extension spring (~15 mm free length, ~0.15 N/mm) *(or rubber band)* | Step 6 |
| 1 | 15 mm velcro strap (~200 mm length) | Step 6 |
| 1 | Sourccey 20 × 20 mm wrist camera + mount | Step 7 |
| 1 | *Optional:* 1/4-20 × 1-1/4" hex bolt (for optional top accessory mount) | Step 7 |
| 2 | Printed 20 × 20 mm ArUco markers (`id 0` & `id 1`) | Step 8 |

### Assembly Sequence

```mermaid
flowchart LR
    S1["1. Print 4 Parts"] --> S2["2. Heat-Set Inserts"]
    S2 --> S3["3. Bolt Grip to Base"]
    S3 --> S4["4. Bolt Lever to Finger"]
    S4 --> S5["5. Install Servo & Finger"]
    S5 --> S6["6. Pin, Spring & Strap"]
    S6 --> S7["7. Wrist Camera"]
    S7 --> S8["8. ArUco & Ergonomics"]
    S8 --> S9["9. Calibrate & Bench Test"]
```

> [!IMPORTANT]
> **Order matters:** Always bolt the **pistol grip onto the base (Step 3)** *before* installing the STS3215 servo (Step 5). Once the servo is seated inside the base, it can block hex-key access to the inner M3 × 20 mm screws.

---

## Step 1 — Print & Inspect the 4 Parts

![Step 1: print 4 parts](img/01_parts.png)

1. Generate the print-ready real Sourccey base and finger STLs (downloaded directly from Vulcan Robotics' repo):
   ```bash
   python tools/prepare_sourccey_parts.py
   ```
2. Slice and print all **4 parts** in **PETG** (`0.2 mm` layers, `0.4 mm` nozzle, `4 perimeters`, `40%` infill; see [`02-printing.md`](02-printing.md) for full slicer settings).
   - **Left arm:** `*_LEFT.stl`
   - **Right arm:** `*_RIGHT.stl`
3. Remove any brims or support material, especially inside the 5-hole counterbores on the base and the spring channel inside the pistol grip beam.

> [!TIP]
> **Dry-fit check:** Thread a spare M3 screw through the 4 holes in the trigger lever and the 5 counterbored holes in the base before heating up your soldering iron. They should pass cleanly without forcing.

---

## Step 2 — Install 5× M3 Heat-Set Inserts in the Grip

![Step 2: M3 heat-set inserts](img/02_inserts.png)

**Parts:** Printed **Pistol Grip** · `5×` **M3 heat-set inserts**

1. Set your soldering iron to **~220 °C** (for PETG).
2. Place one M3 brass insert squarely on top of one of the **5 red-marked Ø4.2 mm holes** on the front interface face of the grip.
3. Press **straight down with light, steady pressure** until the insert sits **flush** with the printed face (6 mm depth). Let the heat melt the plastic—do not force it.
4. Repeat for all 5 holes in the cross pattern.
5. Let the part cool for 2 minutes so the PETG solidifies around the knurling.

> [!WARNING]
> If an insert goes in crooked, re-heat it gently with the iron tip, straighten it, and hold it square until it cools. A tilted insert will bind the M3 × 20 mm screws in Step 3.

**✓ Check:** Lay a straightedge across the 5-hole face—no insert should protrude above the surface.

---

## Step 3 — Bolt the Pistol Grip to the Sourccey Base

![Step 3: grip onto the Sourccey base](img/03_grip_on_base.png)

**Parts:** **Pistol Grip** (with inserts) · **Sourccey Base** · `5×` **M3 × 20 mm screws**

1. Align the 5-hole cross on the rear of the **blue Sourccey base** with the 5 heat-set inserts on the **pistol grip**.
2. Insert the **5× M3 × 20 mm screws from INSIDE the base** (screw heads seated cleanly inside the Ø6.2 mm counterbores) into the grip inserts.
3. Tighten all 5 screws evenly in a cross pattern so the interface pulls completely flush.

> [!NOTE]
> If your Sourccey revision mounts the wrist screws from the opposite direction, reverse the hardware: press the 5 heat-set inserts into the base instead and drive the M3 screws through the grip.

**✓ Check:** Hold the base in one hand and twist the pistol grip firmly with the other. There must be **zero play or flex** at the joint.

---

## Step 4 — Bolt the Trigger Lever to the Sourccey Finger

![Step 4: trigger lever on finger](img/04_lever_on_finger.png)

**Parts:** **Sourccey Finger** · **Trigger Lever** · `4×` **M3 × 16 mm screws**

1. Position the **green trigger lever** flat against the outer face of the **orange finger's `-y` lug** (the side opposite the wrist camera wing).
2. Align the lever's 4-hole cross with the 4 existing holes in the finger lug.
3. Fasten with **4× M3 × 16 mm screws**. *(If your finger already has screws installed on that side, replace them with screws **6 mm longer** to account for the 6 mm lever plate.)*
4. Tighten evenly so the lever sits dead-flush against the finger.

**✓ Check:** Twist the trigger lever against the finger by hand—the lever and finger must behave as a **single rigid piece** with zero wobble.

---

## Step 5 — Assemble Servo, Horn & Finger

![Step 5: assembled gripper](img/05_assembled_gripper.png)

**Parts:** **Base + Grip** assembly · **Finger + Lever** assembly · **Feetech STS3215 servo** + horn + servo mounting screws

1. Install the **Feetech STS3215** servo into the base cavity, attach the servo horn, and mount the **finger + lever** subassembly onto the pivot exactly as specified in the official *Sourccey Assembly Manual* (`vulcan-forge/sourccey-hardware`).
2. Before powering anything, ensure the servo is **unpowered (torque off)** and check that you are using a **1:147 or 1:191 gear-ratio** STS3215 for smooth manual back-drivability.

**✓ Check (Full Range Sweep):** Rotate the finger smoothly by hand from **`-5°` (fully closed)** to **`60°` (fully open)**:
- The lever must clear the base and grip cleanly at every angle (`≥ 4 mm` CAD clearance).
- At **`~58–60°`**, the lever's rear tail tab should land cleanly on top of the grip beam (mechanical open stop).

---

## Step 6 — Install the Anchor Pin, Return Spring & Velcro Strap

![Step 6: pin and spring](img/06_spring_and_pin.png)

**Parts:** `1×` **M3 × 70 mm screw + M3 nut** · `1×` **Extension spring** · `1×` **15 mm velcro strap (~200 mm)**

1. Slide the **M3 × 70 mm screw** (or 3 mm smooth rod) through the transverse pin hole in the grip beam (`x = -42 mm, z = -26 mm`), capturing one hook of the **extension spring** inside the center channel as you pass it through.
2. Secure the far end of the pin with the **M3 nut** (snug—do not crush the beam walls).
3. Using needle-nose pliers, hook the other end of the extension spring through the **Ø3.2 mm hole in the lever's upper tail**.
   - *Sanity check:* The spring should be **short when open (~28 mm at `q = 60°`)** and **stretched when closed (~59 mm at `q = -5°`)**, always pulling the jaws **open**.
4. Thread the **15 mm velcro strap** through the two vertical slots on the sides of the concave green trigger pad, forming a snug adjustable loop for your index finger.

> [!TIP]
> **Why the velcro strap matters:** Unpowered STS3215 servos have gearbox friction. If the spring alone does not snap the jaws all the way back to `60°`, the velcro strap lets your index finger actively push the trigger forward to reopen the gripper effortlessly.

**✓ Check:** Pull the trigger fully closed and release at several intermediate angles. Verify smooth motion and check whether the spring reopens the jaws on its own (record this in [`BENCH_CHECKLIST.md`](../BENCH_CHECKLIST.md)).

---

## Step 7 — Mount the Sourccey 20 × 20 mm Wrist Camera

![Step 7: Sourccey 20x20 mm camera](img/07_camera.png)

**Parts:** **Sourccey 20 × 20 mm camera** + robot mount · *(Optional)* `1×` **1/4-20 × 1-1/4" hex bolt**

1. **Wrist Camera (only camera needed):** Mount the **20 × 20 mm Sourccey camera** behind the **Ø25 mm circular opening** in the blue base's `+y` side wing using the same mount your Sourccey robot uses. Aim it toward the jaws—the UMI parts are ray-cast verified not to block the jaw gap.
2. **Optional Top 1/4-20 Accessory Mount:** No GoPro is needed or recommended (to keep the UMI low-cost). Leave the top pedestal empty for single-camera use, or slide a **1/4-20 hex bolt** sideways into the rear hex slot if you later attach an optional RGB-D/IMU module (e.g., OAK-D SR / Orbbec Gemini 305 for Pollen Robotics' `grabette-slam` Space) at `x = -51, y = -7, z = 34 mm` in the pivot frame.
3. Route the servo bus cable and wrist camera cable neatly down the rear of the pistol grip using zip ties or tape so they do not snag your wrist during demonstrations.

---

## Step 8 — Attach ArUco Markers & Check Ergonomics

![Step 8: how to hold the Sourccey UMI](img/08_how_to_hold.png)

**Parts:** `2×` **20 × 20 mm ArUco stickers** (`software/aruco_id0_20mm_300dpi.png` and `software/aruco_id1_20mm_300dpi.png`)

1. Print both ArUco markers at **100% scale (300 DPI → exact 20 × 20 mm)** and stick them cleanly onto:
   - **`id 0`:** Flat upper surface of the **orange moving finger**
   - **`id 1`:** Flat outer surface of the **blue fixed jaw**
   *(These allow `software/sync_offset.py` to synchronize the tracker video clock with the 200 Hz servo angle stream to sub-millisecond accuracy.)*
2. **Hold the UMI naturally:**
   - Wrap your **palm** behind the dark-gray pistol grip and seat your **middle, ring, and pinky fingers** into the 3 sculpted grooves.
   - Slip your **index finger** into the velcro loop in front of the concave green trigger pad.
   - Pull back smoothly to close the jaws; relax (or gently extend) to reopen.

---

## Step 9 — Verify Operation & Run Calibration

![Step 9: gripper operation](img/09_operation.png)

| Trigger State | Gripper Joint (`q`) | Jaw Gap *(at 55 mm depth)* | Spring Length |
|---|---|---|---|
| **Released (Open Stop)** | `~60°` | `~82 mm` | `28 mm` |
| **Halfway** | `30°` | `~46 mm` | `42 mm` |
| **Fully Pulled (Closed)** | `-5°` | `~10.6 mm` | `59 mm` |

### Next Steps
1. **Wire & Calibrate:** Follow [`04-electronics-and-software.md`](04-electronics-and-software.md) to connect the STS3215 USB bus adapter + 7.4 V supply and run `python software/sts_angle.py calibrate`.
2. **Bench Validation:** Complete every item in [`BENCH_CHECKLIST.md`](../BENCH_CHECKLIST.md) before collecting robot training episodes.

---

## Quick Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| Trigger feels stiff or doesn't reopen to `60°` | High unpowered STS3215 gearbox friction or weak spring | Keep the **velcro finger strap** snug around your index finger, or step up to a slightly stiffer extension spring (~0.20 N/mm). |
| M3 × 20 mm screws won't start into grip | Crooked heat-set insert or support debris in counterbore | Re-heat and square the insert (Step 2); clear counterbores with a 3.2 mm drill bit by hand. |
| Lever scrapes the base or stops before `58°` | Lever screws loose or servo horn offset by 1 spline tooth | Re-seat the servo horn per the Sourccey manual and re-tighten the 4× M3 × 16 mm lever screws. |
| Wrist camera sees blurred/clipped edge | Camera board shifted off-center behind Ø25 mm wing hole | Re-center the 20 × 20 mm PCB behind the wing opening and verify live preview shows both jaws cleanly. |
