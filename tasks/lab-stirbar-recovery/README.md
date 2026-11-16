# Magnetic stir-bar recovery through a flask neck

Implementation candidate. No native success, model score or media is claimed yet.

This is dry-flask recovery after decanting, a real laboratory preparation step. A permanent magnetic retriever captures a free25x7mm stirring bar, and the robot has to guide that coupled assembly through a22mm neck. A transverse bar cannot pass. The tool is mounted before the trial; tool pickup, formulation chemistry and liquid drainage are not tested. The task is intended for laboratory automation under **Other**, not a claim that500,000 workers regularly change electrode-production rolls.

## Real props and reconstruction

The [Cowie300mm retriever](https://www.cowie.com/product/40/polypropylene-stir-bar-retriever) is a standard product,004.1300,10mm diameter. A commercially priced alternative is the [UnitedScientific MBR350](https://www.usplastic.com/catalog/item.aspx?itemid=102394), listed at$25.59; it is350mm long and requires the corresponding tool-length adjustment. The [Corning6970SR retriever](https://www.thomassci.com/p/magnetic-stir-bar-retriever) is listed at$42.76. An ordinary100mL narrow-neck flask and PTFE bar complete an approximately$100 props budget, excluding robot/tax/delivery. No powered stirrer, special proof fixture or precision metrology is needed. Do not leave the vessel on a powered magnetic stirrer while retrieving the bar.

The native scene uses original dimensions in task.json and build.py:56mm internal lower-body diameter, tapered shoulder,22mm internal neck, and rimZ0.855m. These are generic flask geometry, not a claim to reproduce one manufacturer's CAD. Use a flask matching the neck/bar clearances, and measure the actual rim and inner-floor height with a ruler. The supplied USD uses48 convex wall sectors; no convex hull fills the neck or vessel. The full25mm bar and300mm rod are not enlarged or miniaturized.

The simulated permanent field is an explicitly bounded two-pole approximation for each magnet. Four signed, softened pair interactions create attraction, repulsion and the associated orientation torque. Each radial pair force is bounded at0.10N, tapers beyond25mm and vanishes at30mm. Total force is therefore bounded at0.40N. Both bodies receive balanced forces and torques about their actual COMs. There is no nearest-pole switching, attachment, capture constraint, prescribed rotation or pose write. The prototype endpoint-only model failed native retrieval and is preserved as a failed experiment; it is not feasibility evidence for this model.

Before matching the simulator to a particular retriever, measure its near-contact holding force and falloff using a low-force spring scale and separation spacers. Coefficients here are declared benchmark choices, not Corning/Cowie measurements. Native collision walls remain solid regardless of magnetic force.

## Embodiment and score

The robot is a single six-axis Cartesian-gimbal tool manipulator, with native finite-force joints. It is not a joint-accurate commercial-arm model and is not bimanual YAM. Its Cartesian tool trajectory can be reproduced by a real six-axis arm carrying the same smooth retriever. The operation is scored using a video/ruler: partial credit for lifting the retained bar15mm off the bottom or bringing its centre to the rim; full score for the entire retained bar25mm above the rim and retained for2s. The held-tool endpoint is intentional for this retrieval task; a tool moved without the bar earns no extraction credit. Any orientation and feasible path is allowed. Precision magnetic-force readings are not grading requirements.

The flask is empty and at room temperature. No biological material, hazardous solvent, powder or disposable consumable is required. Reset the same dry bar on the bottom and restore the tool position. Keep smooth protective polymer on the rod and approach glass gently.

## Run and evidence

On the pinned Linux Isaac Lab-Arena environment with RTX rendering:

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --no-deps -e tasks/lab-stirbar-recovery
python tasks/lab-stirbar-recovery/run.py smoke --output evidence/stir-smoke-01 --max-steps 200
python tasks/lab-stirbar-recovery/run.py demo --output evidence/stir-demo-01 --max-steps 1200
python tasks/lab-stirbar-recovery/run.py model --output evidence/stir-model-01 --bridge evidence/stir-bridge --max-steps 2400
```

The separate LLM policy requires actual code-aware/non-blind model responses, recorded images, exact instruction and rationale; it never substitutes the demo. Every run records source hashes, runtime versions, native Inspect log, physics trace and complete camera streams. Control time is at most120s; actual wall time is in the manifest. export.py retains every frame at20Hz. Negative controls disable the field without changing geometry. Source-bound model/demo results, setup image and fresh media are mandatory before submission.

Nearest board comparison: TK-TUMETS attaches a magnetic stylus to a tablet, without a free body being reoriented through a hollow-neck constraint. Ordinary pick-and-place and magnet attachment alone are insufficient distinctions. Refresh and inspect current board/suite neighbors before submitting.

Original source and primitive assets are MIT licensed. Dependencies retain their upstream licences.

The retention hold checks the visible outcome, not instantaneous solver velocity. The robot may move the retained bar while keeping it clear; such motion is irrelevant to successful retrieval. Native velocities remain diagnostic only.
