# Secondary-dryer vacuum-readiness fixture: partial implementation

This folder currently implements the reduced gas model and its verification.
It has no native door, hinge, gasket, clamp or valve scene, no demonstrated
full-score trial, and no model evaluation. It is not ready for submission.

Run the offline checks from this folder:

```sh
python -m unittest discover -s . -p 'test_*.py'
```

The intended scope is closure and mechanical vacuum readiness of a full-size
electrode-dryer module. It does not measure electrode dryness or certify an
industrial leak specification. The proposed 20 kPa absolute test and 500 Pa
rise over 10 seconds are benchmark choices requiring physical calibration.

`dryer_vacuum.gas.Chamber` stores air mass and derives absolute pressure from
the ideal-gas law at constant 293.15 K. The caller supplies actual chamber
volume, sixteen measured gasket gaps, actual pump/vent shaft angles and the
physical pump switch. There is no closed-door Boolean or prescribed pressure.

Each seal segment uses compressible laminar parallel-plate leakage,
`mdot = width * gap^3 * (p_high^2 - p_low^2) / (24 * viscosity * length * R * T)`.
The effective gap is its positive measured clearance plus a proposed 2 micrometre
residual channel. Large gaps are capped by finite compressible nozzle flow.
The vent is an 8 mm reduced nozzle. Actual valve angle gives a `sin(angle)^2`
open fraction. Valve conductance, discharge coefficient 0.62, seal path 12 mm,
residual channel and constant effective pump speed all require calibration;
they are not manufacturer performance claims.

Backward-Euler integration solves the mass balance at the new pressure and
volume. Moving boundaries conserve air mass when all paths are closed. The
reported signed load `(ambient - chamber pressure) * door area` must be applied
to the actual native door at its centre, producing force and hinge torque.
For a 0.25 m2 door at 20 kPa absolute this is 20,325 N inward. Merely displaying
this value is insufficient: native mechanical coupling remains to be built.

The checks cover analytical pump-down, mass and pV conservation, pressure-force
sign and magnitude, an individual hidden seal gap, an open door, isolation,
venting, finite sonic flow and time-step convergence. Passing them validates
these mathematical limits, not real seal calibration or task feasibility.

Sources and parameter boundaries:

- [TOB BE-ZK-125L family](https://www.tobmachine.com/512l-vacuum-degassing-oven_p576.html):
  500 x 500 x 500 mm chamber, 125 L and 130 kg whole unit. Proposed 25 kg door
  and latch geometry are custom benchmark choices, not TOB specifications.
- [Leybold TRIVAC B](https://www.leybold.com/en/products/vacuum-pumps/oil-sealed-vacuum-pumps/rotary-vane-pumps-trivac-b):
  D16B pumping speed 16.5 m3/hour at 50 Hz is the reference scale. This model
  uses it as a constant chamber-side speed and omits the real pump curve and
  connecting-line conductance; it is not a pump digital twin.
- [NASA mass-flow choking](https://www.grc.nasa.gov/www/k-12/airplane/mflchk.html):
  ideal-gas compressible nozzle mass flux and its sonic bound.
- [MIT viscous-flow equations](https://web.mit.edu/2.25/www/pdf/viscous_flow_eqn.pdf):
  continuum viscous-flow equations underlying the stated slit-law derivation.
- [BINDER VD115 data sheet](https://www.binder-world.com/fileadmin/shop/1.d/files/6/2/62c0e495_Data-Sheet-Model-VD-115-de.pdf):
  its stated 0.01 bar/hour leakage is much tighter than the proposed gross
  500 Pa/10 s readiness threshold. The latter is not an OEM acceptance test.
