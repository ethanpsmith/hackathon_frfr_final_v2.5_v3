"""Ground-supported mock release with continuous interlocks and explicit uncertainty."""
from dataclasses import dataclass
from enum import Enum
from typing import Protocol
from .core import number, Rejected


@dataclass(frozen=True)
class Evidence:
    timestamp: float
    landed: bool = True
    speed: float = 0.
    altitude: float = 0.15
    range_m: float = 0.15
    range_valid: bool = True
    destination_valid: bool = True
    pad_clear: bool = True
    target_visible: bool = True
    target_id: int = 7
    target_confidence: float = 0.99
    contact: bool = True
    supported_fraction: float = 1.
    latch_closed: bool = True
    latch_open: bool = False
    package_separated: bool = False

    def valid_support(self, now):
        try:
            numeric = (self.timestamp, self.speed, self.altitude, self.range_m, self.target_confidence, self.supported_fraction)
            for x in numeric:
                number(x)
            # Invalid / contradictory digital input is never coerced to truthy.
            digital = (self.landed, self.range_valid, self.destination_valid, self.pad_clear,
                       self.target_visible, self.contact, self.latch_closed, self.latch_open, self.package_separated)
            return (all(type(x) is bool for x in digital)
                    and 0 <= now-self.timestamp <= 0.5
                    and self.landed and 0 <= self.speed <= 0.15
                    and 0 <= self.altitude <= 0.3 and self.range_valid and 0.1 <= self.range_m <= 0.3
                    and abs(self.altitude-self.range_m) <= 0.15
                    and self.destination_valid and self.pad_clear and self.target_visible
                    and type(self.target_id) is int and self.target_id == 7 and 0.9 <= self.target_confidence <= 1.
                    and self.contact and 0.9 <= self.supported_fraction <= 1.1
                    and self.latch_closed != self.latch_open)
        except Rejected:
            return False


class Actuator(Protocol):
    def release(self): ...
    def retain(self): ...


class InertActuator:
    def release(self):
        raise Rejected("release is disabled")

    def retain(self):
        pass


class SimulatedActuator:
    """In-memory events only. No MAVLink servo/gripper or electrical output."""
    def __init__(self):
        self.events = []

    def release(self):
        self.events.append("release")

    def retain(self):
        self.events.append("retain")


class DeliveryState(str, Enum):
    VERIFYING = "VERIFYING"
    OPENING = "OPENING"
    RELEASED = "RELEASED"
    RETAINED = "RETAINED"
    UNCERTAIN = "UNCERTAIN"


class Delivery:
    def __init__(self, now, *, simulate_release=False, phase_authorized=False):
        self.enabled = simulate_release and phase_authorized
        self.actuator = SimulatedActuator() if self.enabled else InertActuator()
        self.state = DeliveryState.VERIFYING
        self.started = now
        self.stable_since = None
        self.opened_at = None
        self.events = []

    def change(self, state, reason):
        self.state = state
        self.events.append(dict(event="delivery_state", state=state.value, reason=reason))

    def abort(self, reason, evidence=None, now=None):
        if self.state in (DeliveryState.RELEASED, DeliveryState.RETAINED, DeliveryState.UNCERTAIN):
            return
        self.actuator.retain()
        # Before actuation, a release was never requested. After actuation, require
        # a fresh closed-latch observation before claiming captive retention.
        known_closed = (evidence is not None and now is not None and evidence.valid_support(now)
                        and evidence.latch_closed and not evidence.package_separated)
        state = DeliveryState.RETAINED if self.opened_at is None and known_closed else DeliveryState.UNCERTAIN
        # Missing pad/weight evidence with independently fresh closed latch still retains.
        if self.opened_at is None and evidence is not None:
            try:
                if 0 <= now-number(evidence.timestamp) <= .5 and evidence.latch_closed is True and evidence.latch_open is False and evidence.package_separated is False:
                    state = DeliveryState.RETAINED
            except (Rejected, TypeError):
                pass
        self.change(state, reason)

    def step(self, e, now):
        if self.state in (DeliveryState.RELEASED, DeliveryState.RETAINED, DeliveryState.UNCERTAIN):
            return self.state
        if not self.enabled:
            self.abort("simulation release disabled", e, now)
        elif now-self.started > 8:
            self.abort("delivery timeout", e, now)
        elif not e.valid_support(now):
            self.abort("missing, stale, or contradictory interlock evidence", e, now)
        elif self.state == DeliveryState.VERIFYING:
            if not e.latch_closed or e.package_separated:
                self.abort("payload not captive before release", e, now)
            else:
                if self.stable_since is None:
                    self.stable_since = now
                if now-self.stable_since >= 1.0:
                    self.actuator.release()
                    self.opened_at = now
                    self.change(DeliveryState.OPENING, "support stable for one second; mock release")
        elif self.state == DeliveryState.OPENING:
            if e.latch_open and e.package_separated:
                self.change(DeliveryState.RELEASED, "fresh latch-open and separation confirmed")
            elif now-self.opened_at >= 2:
                self.abort("latch/separation confirmation timeout", e, now)
                if e.latch_closed and not e.package_separated:
                    self.change(DeliveryState.RETAINED, "fresh closed latch confirms captive payload")
        return self.state

    @property
    def payload(self):
        return {DeliveryState.RELEASED: "separated", DeliveryState.RETAINED: "retained"}.get(self.state, "unknown")
