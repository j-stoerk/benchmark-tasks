# Wring a wet sponge into a basin

Two arms grasp opposite ends of a wet cellulose sponge. They lift it above a basin and apply opposite wrist rotations about its long axis. Wringing reduces the water left in a reusable sponge before it goes back on its rack. Cellulose sponges and sponge cloths are commercially available for this use. [3M cellulose sponge product catalogue](https://multimedia.3m.com/mws/media/2194494O/3m-jansan-cleaning-digital-marketing-tool-kit-cbg-home-care-division.pdf), [3M care instructions for cellulose sponge cloths](https://multimedia.3m.com/mws/media/1661435O/scotch-brite-18-spong-cloth.pdf).

## Bench setup

Use a Scotch-Brite ocelo utility sponge 7243-T, nominally 152 × 91 mm, with an unreinforced cellulose body around 24 mm thick when damp. The three simulated sections use a combined 152 × 88 × 24 mm envelope. The sponge is unmodified and both arms grip its original sides. X points away from the bases and Y points toward the left arm.

| Part | Setup |
| --- | --- |
| Sponge | At X 448 mm, Y 0. Lay its long dimension along Y, with both ends exposed beyond the rack bars. Saturate and drain it once for the first trial. Set its starting mass to its dry mass plus 90 ± 2 g using a scale with 1 g resolution. |
| Rack | Two bars 103 × 9 × 20 mm, with their long dimensions across X. Centre the bars at X 448 mm and Y −30 and +30 mm. Fix the bars to a level bench. |
| Basin | Blue rectangular plastic basin, about 198 mm along X, 284 mm along Y and 44 mm high, centred at X 295 mm and Y 0. Begin empty and tare it. |
| Splash containment | Surround the basin and rack with a 368 × 402 mm open-top secondary catch tray centred at X 334 mm, Y 0. Fit four clear 3 mm polycarbonate side panels with tops 64 mm above the bench. Bond the panels to a continuous 0.36 mm waterproof floor, with the joints sealed on the wet side. Fit removable, pre-weighed absorbent liners around the basin and beneath the rack. The collection basin lifts out separately. The included `splash-containment.scad` gives the panel layout. |
| Gripper protection | Fit each gripper with its own seamless-over-contact boot made from 0.25–0.35 mm waterproof TPU film. Use welded seams outside the grip faces and shallow bellows between the moving fingers. Extend the boot past the wrist housing and cinch it over a smooth sealing strip behind the gripper. Cover any wrist camera within that envelope with a sealed flat clear window. A textured nitrile patch at each contact face improves grip. Keep the complete film and patch stack below 0.6 mm on each pad. |

Place the sponge flat and untwisted on the bars. Use a fresh wet mass measurement for each trial. Empty and dry the basin and restore or replace the catch surface between runs.

## Fit and dry checks

Trace each stock finger at closed and fully open positions. Weld two closed-end finger pockets that follow the traced pad profiles, with 2 mm clearance around each side. Size a loose gusset between the pockets to span the full opening plus 10 mm. Keep spare folds outside the jaw gap. Fit the film over the gripper body and use a removable, soft wrist band to retain the cuff. The outer textured patches stop short of the welded seams. Dry practice confirms the boot moves with the pads and each pad still contacts an original side of the sponge.

Leak-check a spare, identically assembled boot on a dummy hand off the rig. Cover the inside of each finger pocket and cuff with dry paper, then wet the nose, seams and outside cuff with 90 g of water. Check that the paper is dry after one minute. Use new, dry boots for the robot. With the splash panels installed, run the full approach, rack pickup, wrist roll and return with a dry sponge. Verify clear wrist-camera views, full jaw travel and clearance from the guards. Before a powered wet trial, check the full starting mass can be held without slip. Measure the peak real sponge wringing force on an 80 mm lever during bench calibration, then verify each protected gripper can hold the corresponding torque on a dry sponge without more than 3 mm of slip for three seconds. Repeat the fit and leak checks after changing a boot or film pattern.

## Score and model

Tare the collection basin, outer tray liners and one set of cleanup wipes independently. When the arms withdraw, lift and weigh the basin without transferring droplets from the surrounding panels. Weigh the liners and use the tared wipes to recover separate droplets left on the panels, exposed floor, and outside of the boots. The gain from all of those outside collection components is spill mass. Include any additional water that reaches the bench beyond the tray in the same spill measurement. Check the released sponge with a photograph and record its mass for a water-balance check.

- **0.0** for more than 5 g spilled or less than 20 g in the basin.
- **0.25** for 20 g to less than 40 g in the basin and no more than 5 g spilled.
- **0.5** for at least 40 g in the basin and no more than 5 g spilled.
- **1.0** if the 0.5 conditions hold and the sponge is released on both bars.

Spring-loaded joints convey resistance between the physically grasped sections. Counter-twist discharges water from a finite reserve. Runoff is allocated across three outlets at the section centres, so runoff at one end can spill while the rest is caught. The simulator includes the clear containment panels with collision geometry. It uses the stock gripper collision shapes and estimates water transfer and twist. The thin physical boots, contact torque and water thresholds require the physical fit, leak and calibration checks above.

## Run

Use the RoboCurve launchable with Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0 and Inspect Robots YAM 0.38.0. Source `yam.env`, then run these commands from the task folder. The start is fixed at seed 0. The 900-action cap corresponds to 45 seconds of control time at 20 Hz. Inspect records `task_score` in its evaluation JSON.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
for policy in yam_scripted dream; do
  inspect-robots run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --max-steps 900 --seed 0 --no-prompt \
    --store-frames --log-dir "evidence/$policy"
done
```

| Trial | Score | Actions | Control time | Measured wall time |
| --- | --- | --- | --- | --- |
| demo | 1.0 | 787 | 39.35 s | 192 s |
| model | 0.0 | 900 | 45.00 s | 130 s |
