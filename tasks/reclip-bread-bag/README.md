# Close an open bread bag with its reusable retaining clip

One hand gathers and steadies the loose polyethylene neck while the other fits a separate clip over it. The clip should stay on the film after both hands let go.

## Setup and reset

Use the calibrated bimanual YAM station. Measure X and Y from the midpoint of the two arm mounts projected onto the tabletop. Positive X points forward from the arms and positive Y points to the robot's left. Measure heights from the tabletop.

| Item | Specification |
| --- | --- |
| Bag | Clear, unprinted, side-gusseted LDPE bread bag, nominal **203 × 76 × 381 mm** (face width × opened gusset depth × usable length). Use **25 µm (1 mil)** single-wall film. Measure the width with the gussets tucked in and the length from the bottom seal to the mouth. Use the supplier's gauge specification or measure a ten-layer stack from a spare bag and divide by ten. Interplas P-BR-HI1115 is one example. [Bag specification](https://www.interplas.com/products/8x3x15-1mil-plastic-bread-bags-p-br-hi1115?id=253813) |
| Clip | Red, hand-applied Kwik Lok Series A, **21.4 × 34.9 mm**. Use intact clips with the same opening pattern across runs. [Series A closure](https://www.kwiklok.com/product/series-a/) |
| Loaf and stops | 120 × 240 × 90 mm firm foam block, centred at X 367 mm and Y +139 mm with its long axis along Y. Two 9 × 190 × 16 mm side guides at X 298 mm and 436 mm, both centred at Y +170 mm, and a 147 × 9 × 16 mm end stop at Y +268 mm limit loaf movement. Use smooth wood or rigid plastic. |
| Neck rest | Smooth wood or rigid plastic block, 46 × 16 × 61 mm, centred at X 357 mm and Y +7 mm. Round exposed edges that touch film. |
| Clip stand | Smooth wood or rigid plastic block, 44 × 25 × 32 mm, centred at X 335 mm and Y −195 mm. Two 43 × 4 × 3 mm strips on top leave a 2 mm slot along X. Set the clip upright in the slot with its entry facing negative X. |
| Reset gauge | Smooth rounded plug with a 20 mm wide × 6 mm high cross-section. Use `mouth-gauge.scad` for a printable plug or make an equivalent one from smooth plastic. Keep the gauge off the station during the run. |

1. Insert the loaf with the bag opening toward negative Y. Set the mouth edge **102 ± 2 mm** beyond the loaf face at Y +19 mm. With the loaf in position, the mouth is at Y −83 ± 2 mm. Fold the spare length at the sealed bottom under the far end of the loaf, away from the working neck.
2. Rest the film on the neck block. Preserve the side gussets. Bring the loose film inward around the gauge instead of twisting it. Insert the plug 15 mm into the mouth with its 20 mm dimension along X. Smooth the film lengthwise toward the loaf and pinch loose excess into relaxed folds beside the plug. Slide the plug straight out toward negative Y. The lips should be separated at the centre and the opening should span **20 ± 2 mm along X** with a **4–8 mm** vertical gap. Check from above and from the mouth end with a ruler beside the film. Re-form the opening if it collapses or falls outside either band before the start.
3. Seat the clip in its stand. Take a start photo from above and another from the mouth end, keeping the ruler outside the robot's work area. Reset the positions and opening between trials. Replace a bag with holes, split seams or stretched film at the neck. Use a fresh bag if a permanent crease prevents the opening check from passing.

## Score

Use the highest condition satisfied at the end of the trial. Record the final two seconds from above and from the mouth end. Place a millimetre scale beside the working area without touching the film or clip. Inspect both faces in close-up. Trace film from the loaf side, through the enclosed hole behind the entry teeth, to the free tail. Measure the width along the film itself, with at least 8 mm extending from each face. A strand lying in the entry slit or on the outside of a clip face does not count.

| Score | Observable outcome |
| --- | --- |
| **0.0** | The neck has a new tear or a detached piece, or no continuous captured span at least 2 mm wide meets the aperture and tail check. |
| **0.25** | The neck is intact and a continuous span at least **2 mm wide** passes through both faces with at least **8 mm** of tail. Some neck film still bypasses the hole, including film caught in the slit. |
| **0.5** | The intact neck passes the aperture and tail check across its entire width. Follow both bag walls and tucked gussets from the loaf to the tail to verify that none bypasses the hole or exits through the slit. The release or hold conditions for full credit are still unmet. |
| **1.0** | The entire intact neck passes the previous check. The loaf sits on the bench within **10°** of upright, the clip centre is more than **24 mm** beyond the loaf's mouth-facing end along negative Y, and both fingertips are more than **60 mm** from the clip centre. These conditions hold for the final **2 s**. During that interval, the clip and loaf stay within **1.5 mm** of their positions at its start and the clip turns by no more than **5°**. Use a ruler or a grid in the video to check position and rotation. |

The simulator represents the neck with articulated contact strips and uses sprung teeth at the clip entry. The scorer clips the film's cross-section against both aperture faces and measures continuous captured material width. The simulated film thickness is a collision proxy. Actual film friction, clip force and the reset gauge still require bench characterization.

## Run

Use the RoboCurve launchable and source `yam.env`. From the task directory run the following command for each listed policy.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
for policy in yam_scripted dream; do
  inspect-robots run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --max-steps 680 --seed 0 --no-prompt \
    --store-frames --log-dir "evidence/$policy"
done
```

| Trial | Score | Actions | Control time | Measured wall time |
| --- | --- | --- | --- | --- |
| demo | 1.0 | 376 | 18.80 s | 87 s |
| model | 0.0 | 680 | 34.00 s | 140 s |
