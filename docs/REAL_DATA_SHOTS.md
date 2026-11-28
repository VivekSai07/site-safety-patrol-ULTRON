# Shot list for the next B2 recording (demo room)

What to record with the B2 in the demo room so the perception risks below can be measured
instead of guessed. Every item names the rule in the code it would settle. Today the evidence is:
one hall recording (no lying ladder, no glass wall, no wooden ladder), 7 phone photos of a
likely demo room (different camera, about 1.5 m up, no depth) and rendered tests.

## How to record

Use the relay's recorder (`scripts/ros_relay.py --record DIR`, [RECORDING.md](RECORDING.md)) or
`ros2 bag record` with at least these topics, all in one bag per item:

| Topic | Why |
|---|---|
| `/camera_depth/head/color/image_raw` + `camera_info` | detector; 1280x720 as in the hall bag |
| `/camera_depth/head/depth/image_rect_raw` + `camera_info` | range, floor plane, `lift.doubt` |
| `/camera_depth/head/extrinsics/depth_to_color` | depth registration |
| `/scan_cloud` | lidar candidates (`bag:...,lidar=1`), lidar odometry |
| `/tf`, `/tf_static`, `/imu/data` | pose, and a TF pose if localisation runs |
| the localisation pose (`map` <- `base_link`) if it runs | objects in the map frame |

Walk slowly (about 0.3 m/s, as the hall recording). Say each item's number into the
microphone or hold up a sheet with it in the first frame, and note the time. 20-60 s per item
is enough; the whole list is about 15 minutes of recording (about 100 GB at the hall rate of
7 GB a minute without compression: split the bag, or drop the colour rate to 15 Hz).

Also note, per item, the true positions (tape measure from two walls) of every ladder, so
footprints can be scored.

## 1. Ladder lying on the floor (critical: driving over one)

Settles `lift.flat_on_floor` (lying test, `LYING_TOP`, `RAISED`), the extra detector prompt in
`catalog.ALSO` and its threshold, the detector input size (640 against 1280), and
`discover.LidarCandidates.find_lying`.

1. The wooden extension ladder lying on the carpet, robot standing still facing it, at 1, 2, 3
   and 4 m from its nearest point: across the view (side-on), end-on (pointing away), and at
   about 45 degrees. 5 s per stop. 12 stops.
2. The same, walking past it at 1.5 m and at 3 m (so the lidar rings sweep across it).
3. An aluminium ladder lying flat, the same stops at 2 and 4 m, side-on and end-on.
4. A stepladder folded and lying on its side (it is 0.4-0.6 m high: does it count as lying or
   standing?), at 2 m.
5. A lying ladder half hidden behind a chair or table leg, at 2 m.
6. The empty carpet where the ladder lay (move it away), walked the same way: false lying
   candidates from the lidar and false "lying" sightings from the camera.
7. Low things that are not ladders: a cable cover, a door mat, a box 20 cm high, at 2 m.

## 2. Glass wall, reflections, light (critical: objects or free space behind the glass)

Settles `lift.doubt` (`MIN_VALID`, the sill test), whether the depth camera reads the glass, the
room behind it, or the mirror image, and how the fence must be drawn.

1. The glass wall with nothing in front, robot at 1, 2, 3 and 5 m from it, facing it square
   and at 30 and 60 degrees. 5 s each.
2. The same with an A-frame ladder 1 m in front of the glass (its reflection is visible), and
   with a ladder 3 m in front of the glass (reflection deep in the glass), from 2 and 4 m.
3. A ladder standing directly against the glass (touching the radiator band), from 2 m.
4. A walk along the glass wall at 1.5 m, the whole length.
5. Items 1 and 2 again at another time of day (daylight from behind the glass, or the other
   room's lights on and off), if the room's lighting changes during the event.
6. Lidar: one slow loop of the room, to see how much of the space behind the glass the lidar
   marks as free (this is what the fence must cut off).

## 3. Wooden and aluminium ladders standing

Settles which ladder prompt `calibrate` picks for the room (YOLOE barely separates the
materials: on aluminium ladders "wooden ladder" scored higher than "ladder").

1. Each ladder in the room standing, seen from 2 and 4 m, from two sides. Include the Layher
   stepladder with the coloured marks and the low step stool.

## 4. Tight passages and one-sided access

Settles view goals and the goal gate (`coverage.py`, the fence) and whether footprints of
ladders stay apart in a narrow space.

1. The narrowest gap the robot can pass between two ladders, or a ladder and a wall: walk it
   both ways.
2. A ladder that can be approached from one side only (against a wall or in a corner): stand
   in front of it at 1, 2 and 3 m, and walk the reachable arc around it.
3. Two ladders 0.3-0.5 m apart edge to edge, seen together and one at a time.

## 5. A full patrol

One complete loop of the room as the demo will run it (start, every ladder, back), 2-3 minutes,
with everything in place. This is the regression recording: the pipeline is run on it after
every change.

## What each recording answers

| Item | Question | Code it tunes |
|---|---|---|
| 1.1-1.3 | Does the camera find a lying ladder from the robot's height, at which range and input size? Is the footprint where the tape says? | `catalog.ALSO`, `imgsz`, `lift.flat_on_floor` |
| 1.2, 1.6, 1.7 | Does the lidar see 5-15 cm off the carpet; how many false lying blobs? | `discover.find_lying` (`flat`, `FLAT_SHARE`) |
| 2.1 | Glass: holes, the room behind, or the mirror depth? | `lift.MIN_VALID`, the sill test |
| 2.2-2.4 | Are reflections detected, and does `doubt` drop them without dropping the real ladder in front? | `lift.doubt`, verifier |
| 2.6 | Where does the lidar see free space behind the glass? | fence polygon |
| 3.1 | Which prompt does calibration pick here? | `calibrate`, `catalog` |
| 4.1-4.3 | Are view goals reachable; do close ladders stay two objects? | `coverage`, `objects` |
| 5 | End to end | everything |
