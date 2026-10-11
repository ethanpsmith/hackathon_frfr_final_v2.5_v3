"""Seeded, synthetic device observations. Not validated manufacturer models."""
from dataclasses import dataclass, replace
import random
from .delivery import Evidence, Delivery, DeliveryState


@dataclass(frozen=True)
class Sample:
    timestamp: float
    value: object
    valid: bool
    source: str = "synthetic"


class SensorModels:
    def __init__(self, seed=650, noise=.015, latency=.05, dropout=0.):
        self.rng = random.Random(seed)
        self.noise, self.latency, self.dropout = noise, latency, dropout

    def sample(self, value, now, valid=True):
        return Sample(now-self.latency, value, valid and self.rng.random() >= self.dropout)

    def rangefinder(self, height, now, failed=False):
        return self.sample(height+self.rng.gauss(0, self.noise), now, not failed and .1 <= height <= 12.)

    def gnss(self, north, east, now, drift=0., lost=False):
        return self.sample((north+drift+self.rng.gauss(0,.3), east+self.rng.gauss(0,.3)), now, not lost)

    def compass(self, heading, now, interference=False):
        return self.sample((heading+self.rng.gauss(0,1.)) % 360, now, not interference)

    def camera(self, now, visible=True, target_id=7):
        # Feature-level observation; no synthetic pixels or actual camera driver.
        return self.sample(dict(target_id=target_id, confidence=.99 if visible else 0., north_offset=0., east_offset=0.), now, visible)

    def battery(self, now, remaining_ah=8., current=35., resistance=.035):
        soc = max(0., min(1., remaining_ah/10.))
        return self.sample(dict(voltage=6*(3.3+.9*soc)-current*resistance, current=current,
                                remaining_ah=remaining_ah, remaining_wh=remaining_ah*22.2), now)

    def links(self, now, rc=True, telemetry=True):
        return self.sample(dict(rc_ok=rc, telemetry_ok=telemetry), now)


DELIVERY_SCENARIOS = ("nominal", "release_disabled", "rangefinder_failure", "camera_target_loss",
                      "blocked_pad", "missing_pad", "false_contact", "absent_weight_transfer",
                      "jammed_latch", "contradictory_latch", "failed_release", "stale_sensors", "abort", "timeout")


def delivery_scenario(name, *, enable=False, landed=True, speed=0., altitude=.15):
    if name not in DELIVERY_SCENARIOS:
        raise ValueError(name)
    d = Delivery(0., simulate_release=enable and name != "release_disabled", phase_authorized=True)
    models = SensorModels(noise=.005)
    for i in range(100):
        now = i/10.
        r = models.rangefinder(.15, now, name == "rangefinder_failure")
        cam = models.camera(now, name != "camera_target_loss")
        e = Evidence(now-.05, landed=landed, speed=speed, altitude=altitude, range_m=r.value,
                     range_valid=r.valid, target_visible=cam.valid,
                     destination_valid=name != "missing_pad", pad_clear=name != "blocked_pad",
                     contact=name != "absent_weight_transfer", supported_fraction=0. if name in ("false_contact", "absent_weight_transfer") else 1.)
        if name == "contradictory_latch":
            e = replace(e, latch_open=True)
        if name == "stale_sensors":
            e = replace(e, timestamp=now-3)
        if d.state == DeliveryState.OPENING and name != "jammed_latch":
            e = replace(e, latch_closed=False, latch_open=True, package_separated=name != "failed_release")
        if name == "abort":
            d.abort("operator abort", e, now)
        elif name == "timeout":
            d.step(replace(e, timestamp=9.), 9.)
        else:
            d.step(e, now)
        if d.state in (DeliveryState.RELEASED, DeliveryState.RETAINED, DeliveryState.UNCERTAIN):
            break
    return d
