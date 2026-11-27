# Bill of materials

Prices: I only estimated the servo (and flagged it). Everything else is unchecked. `bom/bom.csv` has the same list as CSV.

| Item | Qty | Specification | Notes | Required |
|---|---|---|---|---|
| PETG filament | ~250 g | 0.2 mm layers, 4 perimeters, 40% infill | the 4 printed parts weigh ~180 g, plus supports | yes |
| Feetech STS3215 servo 7.4 V | 1 | the same one the Sourccey uses; prefer a 1:147 or 1:191 gear ratio | less friction when moved by hand. Estimated US$ 15 to 25 (price not checked) | yes |
| M3 heat-set insert | 5 | M3, 4.2 mm hole | in the front face of the grip | yes |
| M3x20 screw | 5 | pan head | base-to-grip interface (from inside the base) | yes |
| M3x16 screw | 4 | pan head | lever onto the finger, -y side (if screws are already there, use ones 6 mm longer) | yes |
| M3x70 screw + M3 nut | 1 | or a 3 mm smooth rod | spring anchor pin, goes through the beam | yes |
| Extension spring | 1 | free length between hooks ~15 mm; ~0.15 N/mm; extends up to ~45 mm | alternative: rubber band | yes |
| Velcro strap 15 mm | ~200 mm |  | index finger strap (required if the servo has friction) | yes |
| Sourccey camera | 1 | 20 x 20 mm module | the same one as the robot; sits behind the 25 mm opening in the base wing. Board mount: use your robot's own | yes |
| USB adapter for Feetech servos | 1 | e.g. Waveshare Bus Servo Adapter or Feetech URT-1 |  | yes |
| 7.4 V power supply | 1 | for the servo (torque off, read-only) |  | yes |
| 1/4-20 x 1-1/4 in bolt | 1 | head slides in sideways | optional accessory mount on top of the grip (leave empty when using only the 20x20 mm camera) | no |
| Optional RGB-D / pose sensor | 1 | e.g. OAK-D SR or Orbbec Gemini 305 (for Pollen `grabette-slam` cloud RGB-D SLAM) or external tracker | NOT required for single-camera recording; optional mount at x=-51 y=-7 z=34 mm (pivot frame) | no |
| ArUco stickers 20 x 20 mm | 2 | software/aruco_id0_*.png and aruco_id1_*.png | finger (id 0) and fixed jaw (id 1), to sync clocks | for data collection |
| Zip ties / tape | ~4 |  | cable management | no |
| Sourccey base and finger | 1+1 | real Vulcan STLs (downloaded by tools/prepare_sourccey_parts.py) | for the right arm use the v1_2 versions | yes |

Have 5 to 10 spare M3 screws and a small hex key on hand. The servo needs its own ID (see `docs/04-electronics-and-software.md`).
