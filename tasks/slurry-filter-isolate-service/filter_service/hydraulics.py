"""Reduced water circuit with an explicit gas accumulator and geometric slit leaks.

Water is incompressible and isothermal at 20 C. The separate 50 mL gas bladder
is isothermal; its pressure-volume law supplies the circuit compliance. These
are specified fixture assumptions, not an empirical model of battery slurry.
"""

import math

ATM = 101325.0
RHO = 998.2
MU = 0.001002
BLADDER_VOLUME_AT_ATM = 50e-6
BORE = 0.100
COVER_AREA = math.pi * BORE**2 / 4
SUPPLY_GAUGE = 50000.0
RETURN_GAUGE = 30000.0
RESTRICTOR_AREA = math.pi * 0.002**2 / 4
DISCHARGE_COEFFICIENT = 0.62
SEAL_PATH_LENGTH = 0.010


def orifice(up, down, area):
    """Signed Bernoulli flow, including reverse return-line pressurization."""
    dp = float(up) - float(down)
    return math.copysign(DISCHARGE_COEFFICIENT * max(0, area) * math.sqrt(2 * abs(dp) / RHO), dp)


def valve_area(angle_deg):
    """Declared ideal metering trim: zero at closed, full bore at 90 degrees."""
    angle = math.radians(min(90, max(0, float(angle_deg))))
    return RESTRICTOR_AREA * math.sin(angle) ** 2


def seal_flow(pressure, gaps):
    """Geometric thin-slit flow limited by inertial orifice flow at large gaps.

    `gaps` are actual cover-to-gasket surface separations, evenly spaced around
    the circumference. A compressed ideal smooth face has no open channel.
    Surface roughness, porous gasket permeation and chemical wetting are outside
    this gross mechanical sealing model, so it cannot certify leak tightness.
    """
    if pressure <= 0 or not gaps:
        return 0.0
    width = math.pi * BORE / len(gaps)
    flow = 0.0
    for gap in gaps:
        h = max(0, float(gap))
        laminar = width * h**3 * pressure / (12 * MU * SEAL_PATH_LENGTH)
        inertial = orifice(pressure, 0, width * h)
        # The lower asymptotic flow bound prevents the laminar model predicting
        # unrealistically large discharge once inertial losses dominate.
        flow += min(laminar, inertial)
    return flow


class WaterLoop:
    def __init__(self, gauge=SUPPLY_GAUGE):
        self.gas_volume = BLADDER_VOLUME_AT_ATM * ATM / (ATM + float(gauge))
        self.elapsed = 0.0

    @property
    def gauge(self):
        return ATM * (BLADDER_VOLUME_AT_ATM / self.gas_volume - 1)

    @property
    def cover_force(self):
        return max(0, self.gauge) * COVER_AREA

    def update(self, angles, gaps, dt, volume_change_m3=0.0):
        """Advance from actual inlet/return/vent angles and measured seal gaps."""
        if not math.isfinite(dt) or not 0 < dt <= 0.01:
            raise ValueError("Hydraulic step must be finite and in (0,10 ms]")
        if len(angles) != 3 or not all(math.isfinite(float(a)) for a in angles):
            raise ValueError("Three finite actual valve angles are required")
        if not all(math.isfinite(float(g)) for g in gaps):
            raise ValueError("Seal separations must be finite")
        if not math.isfinite(volume_change_m3):
            raise ValueError("Moving-boundary volume change must be finite")
        p = self.gauge
        inlet = orifice(SUPPLY_GAUGE, p, valve_area(angles[0]))
        backflow = orifice(RETURN_GAUGE, p, valve_area(angles[1]))
        vent = orifice(p, 0, valve_area(angles[2]))
        leak = seal_flow(p, gaps)
        change = (-inlet - backflow + vent + leak) * dt + volume_change_m3
        # A moving cover changes actual node volume and therefore pressure;
        # applied pressure work must not be powered by a frozen volume state.
        # At an open atmospheric vent/leak, draining cannot create a vacuum.
        maximum = BLADDER_VOLUME_AT_ATM if angles[2] > 0 or any(g > 0 for g in gaps) else float("inf")
        proposed = self.gas_volume + change
        if proposed <= 0:
            raise ValueError("Hydraulic boundary swept through the entire gas compliance volume")
        self.gas_volume = min(maximum, proposed)
        self.elapsed += dt
        return {
            "gauge_Pa": self.gauge,
            "cover_force_N": self.cover_force,
            "inlet_m3_s": inlet,
            "return_m3_s": backflow,
            "vent_m3_s": vent,
            "seal_leak_m3_s": leak,
            "gas_volume_m3": self.gas_volume,
        }
