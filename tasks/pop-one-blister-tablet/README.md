# Pop one tablet through a blister pack into a cup

Open one push-through pocket while keeping the remaining pockets sealed. Hold the card against the lip of the cup with one gripper while directing the other gripper's pressure through the dome and into the tablet. The foil ruptures under the tablet, which has to fall into the cup. Single-pocket rupture with a clean transfer is the physical objective. [Study of push-out mechanics](https://www.jstage.jst.go.jp/article/cpb/72/12/72_c24-00572/_html/-char/en).

## Setup and reset

Use four adjacent unopened push-through foil pockets of an inert training blister strip, with rounded tablets around 10 mm wide and about 3 mm thick. Wallcur 115TC provides connected four-pocket strips. Wallcur 643FE supplies round inert tablets in foil packs. Confirm the foil is the push-through type and compare the dimensions of the purchased lot with the table below. [Wallcur four-pocket strips](https://www.wallcur.com/Products/Practi-Combat-Pill-Pack-%28for-training%29__115TC.aspx), [Wallcur round training tablets](https://www.wallcur.com/Products/Practi-Iron-65-mg-Oral-Med-%28for-training%29__643FE.aspx).

| Part | Specification and placement |
| --- | --- |
| Pack | Approximately 144 × 23 mm. The simulated pockets are centred on the long axis at −61, −31, −1 and +29 mm relative to the card centre. Leave an intact flat tab on the other end. Mark the clear rim of the outside pocket red without covering the dome. Place foil-down at X 360 mm, Y 141 mm. The free tab faces +Y. |
| Supports | Two square-edged 25 × 6 × 22 mm wooden or printed blocks. Centre at X 360 mm and at Y 72 and 173 mm, long edges across the pack. Fix the blocks to the bench so the card can be picked from above and returned without a clamp. |
| Cup | Blue ceramic condiment cup, approximately 62 mm outside diameter and 34 mm tall, centred at X 360 mm, Y −76 mm. The opening points up and the cup sits on a non-slip mat. Its lip supports the strip during the press. |

Use a fresh four-pocket strip and an empty cup for each trial. Use a strip with at least 20 mm of flat material at the grip end. The X axis runs away from the arm bases and the Y axis toward the left arm. Compare the initial setup with a photo after the arms withdraw. Use the tablet count in the cup and inspect the backing of the three spare pockets under ordinary light. Verify that the strip again spans both supports.

A training fixture can reproduce the specified spacing from individual sealed pockets. Cut a 144 × 23 × 1.2 mm polypropylene strip and four 13 × 13 mm apertures at the coordinates above. Cut four units from a training card with at least 3 mm of its original heat-seal flange still attached around each dome. Align the domes and secure the flanges to the carrier with narrow double-sided tape. Leave the inside of each aperture and its foil uncovered. The combined prop holds and opens the original sealed pockets. Use a spare pocket to check that the carrier leaves the original foil rupture force and release path intact.

## Score and fidelity

- **0.0** if any other blister was opened or the marked tablet is anywhere outside the cup.
- **0.5** for the marked tablet resting in the cup while all three spare pockets are sealed.
- **1.0** if the 0.5 conditions hold and the card is released across the two supports.

The simulator uses a 14 N breakable foil bond on a rigid dummy tablet. A roughly 2 N spring preload at tablet contact precedes foil rupture. The cap physically transmits force and the released tablet falls into the cup as a rigid body. The card and cap geometries omit creases in the plastic film and foil shards. The force value is an initial design target for a thin push-through assembly. To compare a physical strip, support its rim over a cutout and use a small force gauge with a rounded probe to measure the peak on a spare pocket. Cavity depth, shape and batch can change the rupture force. [Experimental measurement method](https://www.jstage.jst.go.jp/article/cpb/72/12/72_c24-00572/_html/-char/en).

## Run

Use the RoboCurve launchable with Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0 and Inspect Robots YAM 0.38.0. After sourcing `yam.env`, run from this directory. Seed 0 sets a fixed starting layout. The 600-action limit is 30 s of control time at 20 Hz. Inspect records `task_score` in the evaluation JSON.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
for policy in yam_scripted dream; do
  inspect-robots run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --max-steps 600 --seed 0 --no-prompt \
    --store-frames --log-dir "evidence/$policy"
done
```

| Trial | Score | Actions | Control time | Measured wall time |
| --- | --- | --- | --- | --- |
| demo | 1.0 | 436 | 21.80 s | 85 s |
| model | 0.0 | 600 | 30.00 s | 109 s |
