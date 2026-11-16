"""Conservative 0D water/air service circuit, not a slurry or seal certificate.

Four measured valves control supply, return, top air vent and bottom drain.
The measured cavity capacity changes with the native cover and immersed solid
volume. Water and air mass are state variables; pressure is an ideal-gas result.
Backward Euler resolves fast vent transients without discarding fluid mass.
"""

import math

from .hydraulics import (
    ATM,
    BORE,
    COVER_AREA,
    DISCHARGE_COEFFICIENT,
    RESTRICTOR_AREA,
    RETURN_GAUGE,
    RHO,
    SEAL_PATH_LENGTH,
    SUPPLY_GAUGE,
    orifice,
    valve_area,
)

R_AIR = 287.05
TEMPERATURE = 293.15
GAMMA = 1.4
MU_AIR = 1.81e-5
GRAVITY = 9.81
RT = R_AIR * TEMPERATURE
LIQUID_RESTRICTOR_DIAMETER = 0.006
DRAIN_DIAMETER = 0.010
PORT_AREAS = [
    math.pi * diameter**2 / 4
    for diameter in (LIQUID_RESTRICTOR_DIAMETER, LIQUID_RESTRICTOR_DIAMETER, 0.002, DRAIN_DIAMETER)
]


def air_flow(upstream, downstream, area):
    """Signed kg/s, isentropic nozzle with a declared 0.62 discharge factor."""
    if not all(math.isfinite(x) for x in (upstream, downstream, area)):
        raise ValueError("Finite absolute pressures and area are required")
    if min(upstream, downstream) <= 0 or area < 0:
        raise ValueError("Absolute pressure must be positive and area nonnegative")
    if upstream == downstream or area == 0:
        return 0.0
    sign = 1 if upstream > downstream else -1
    high, low = max(upstream, downstream), min(upstream, downstream)
    critical = (2 / (GAMMA + 1)) ** (GAMMA / (GAMMA - 1))
    ratio = max(low / high, critical)
    factor = 2 * GAMMA / (GAMMA - 1) * (ratio ** (2 / GAMMA) - ratio ** ((GAMMA + 1) / GAMMA))
    return sign * DISCHARGE_COEFFICIENT * area * high * math.sqrt(max(0, factor) / RT)


def air_seal_flow(absolute, gaps):
    """Signed isothermal slit mass flow, limited by compressible nozzle flow."""
    if not gaps:
        return 0.0
    width = math.pi * BORE / len(gaps)
    total = 0.0
    for gap in gaps:
        h = max(0.0, gap)
        if h == 0:
            continue
        viscous = width * h**3 * (absolute**2 - ATM**2) / (24 * MU_AIR * SEAL_PATH_LENGTH * RT)
        inertial = air_flow(absolute, ATM, width * h)
        total += math.copysign(min(abs(viscous), abs(inertial)), absolute - ATM)
    return total


class ServiceLoop:
    """A custom water service rig with an explicit, ventable air headspace.

    `capacity_m3` is total gas-plus-water volume after subtracting immersed
    solid volume. The initial air space is 50 mL. The cylinder cross-section
    approximates the free-surface height for bottom-port hydrostatic pressure.
    Ports remain connected to the liquid while draining; the top vent supplies
    air. Two-phase flow through process lines and filtration efficiency are
    excluded. No native fixture has yet been validated against this model.
    """

    def __init__(self, capacity_m3=0.0015, gauge=SUPPLY_GAUGE, headspace_m3=50e-6):
        if not 0 < headspace_m3 < capacity_m3 or ATM + gauge <= 0:
            raise ValueError("Positive cavity, headspace and absolute pressure required")
        self.capacity = float(capacity_m3)
        self.water = self.capacity - headspace_m3
        self.vent_liquid_level_volume = self.water
        self.air_mass = (ATM + gauge) * headspace_m3 / RT
        self.elapsed = 0.0

    @property
    def absolute(self):
        return self.air_mass * RT / (self.capacity - self.water)

    @property
    def gauge(self):
        return self.absolute - ATM

    @property
    def cover_force(self):
        # Vacuum also loads the cover: inward load must not be silently erased.
        return self.gauge * COVER_AREA

    def update(self, angles, gaps, dt, capacity_m3=None, vent_liquid_level_volume_m3=None):
        if not math.isfinite(dt) or not 0 < dt <= 0.01:
            raise ValueError("Time step must lie in (0,10 ms]")
        if len(angles) != 4 or not all(math.isfinite(float(a)) and 0 <= a <= 90 for a in angles):
            raise ValueError("Supply, return, top vent and bottom drain angles must be in 0..90 degrees")
        if not gaps or not all(math.isfinite(float(g)) for g in gaps):
            raise ValueError("Finite actual seal separations are required")
        capacity = self.capacity if capacity_m3 is None else float(capacity_m3)
        if not math.isfinite(capacity) or capacity <= 0:
            raise ValueError("Positive finite measured cavity volume required")
        vent_level = (
            self.vent_liquid_level_volume
            if vent_liquid_level_volume_m3 is None
            else float(vent_liquid_level_volume_m3)
        )
        if not math.isfinite(vent_level) or not 0 < vent_level < capacity:
            raise ValueError("Measured liquid capacity below the top bleed port must lie inside the cavity")
        old_water, old_air = self.water, self.air_mass
        # Hydrostatic height uses the beginning-of-step free surface. Native
        # integration must use a small step and check step-size convergence.
        hydrostatic = RHO * GRAVITY * old_water / COVER_AREA
        areas = [valve_area(a) / RESTRICTOR_AREA * area for a, area in zip(angles, PORT_AREAS, strict=True)]
        # At the port's lower lip the headspace is still connected to air;
        # require positive submergence before changing the discharged phase.
        wet_bleed = old_water > vent_level + 1e-12

        def candidate(absolute):
            bottom_gauge = absolute - ATM + hydrostatic
            flows = [
                orifice(SUPPLY_GAUGE, bottom_gauge, areas[0]),
                orifice(RETURN_GAUGE, bottom_gauge, areas[1]),
                -max(0, orifice(bottom_gauge, 0, areas[3])),
                -max(0, orifice(absolute - ATM, 0, areas[2])) if wet_bleed else 0.0,
            ]
            incoming = sum(max(0, flow) for flow in flows)
            outgoing = -sum(min(0, flow) for flow in flows)
            # At empty, a drain cannot remove nonexistent water. Apply the
            # available-volume bound to every outward liquid path together.
            scale = min(1.0, (old_water / dt + incoming) / outgoing) if outgoing else 1.0
            flows = [flow if flow >= 0 else flow * scale for flow in flows]
            water = max(0.0, old_water + dt * sum(flows))
            gas_volume = capacity - water
            # A submerged bleed discharges water instead of draining gas mass
            # into a zero-volume headspace during refill. Once exposed, the
            # same port vents/admit air. Under vacuum it can admit air bubbles.
            air_area = areas[2] if not wet_bleed or absolute < ATM else 0.0
            mass_out = air_flow(absolute, ATM, air_area) + air_seal_flow(absolute, gaps)
            air = old_air - dt * mass_out
            # PV=mRT at the end of this implicit mass-conservation step.
            residual = absolute * gas_volume - air * RT
            if gas_volume <= 0:
                residual = -abs(residual) - 1
            return residual, water, air, flows, mass_out

        low, high = 1e-3, max(2 * ATM, 2 * self.absolute)
        if candidate(low)[0] >= 0:
            raise RuntimeError("No positive-pressure conservative circuit solution")
        for _ in range(40):
            if candidate(high)[0] > 0:
                break
            high *= 2
        else:
            raise RuntimeError("No positive gas volume: measured cavity compressed through stored liquid")
        for _ in range(60):
            middle = (low + high) / 2
            if candidate(middle)[0] > 0:
                high = middle
            else:
                low = middle
        pressure = (low + high) / 2
        residual, water, air, flows, mass_out = candidate(pressure)
        if air <= 0 or water >= capacity or abs(residual) > 1e-7:
            raise RuntimeError("Circuit failed physical state or conservation checks")
        self.water, self.air_mass, self.capacity = water, air, capacity
        self.vent_liquid_level_volume = vent_level
        self.elapsed += dt
        return {
            "gauge_Pa": self.gauge,
            "cover_force_N": self.cover_force,
            "water_volume_m3": water,
            "air_mass_kg": air,
            "capacity_m3": capacity,
            "supply_m3_s": flows[0],
            "return_m3_s": flows[1],
            "drain_m3_s": -flows[2],
            "bleed_water_m3_s": -flows[3],
            "air_out_kg_s": mass_out,
            "free_surface_height_m": water / COVER_AREA,
            "PV_minus_mRT_J": residual,
        }
