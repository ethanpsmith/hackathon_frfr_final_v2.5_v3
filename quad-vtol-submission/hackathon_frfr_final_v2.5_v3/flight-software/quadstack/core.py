"""Pure mission policy; all times use the caller's monotonic clock."""
from dataclasses import dataclass
from enum import Enum
import hashlib
import hmac
import json
import math
import secrets


class Rejected(ValueError):
    pass


def number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise Rejected("expected finite number")
    return float(value)


@dataclass(frozen=True)
class Point:
    north: float
    east: float
    up: float

    @classmethod
    def parse(cls, value):
        if not isinstance(value, list) or len(value) != 3:
            raise Rejected("waypoint must be [north, east, up]")
        return cls(*(number(x) for x in value))

    def distance(self, other):
        return math.dist((self.north, self.east, self.up),
                         (other.north, other.east, other.up))


def to_gps(point, latitude=-35.363261, longitude=149.165230):
    """Local tangent approximation, restricted to this <=100 m demo envelope."""
    return (latitude + math.degrees(point.north / 6378137.0),
            longitude + math.degrees(point.east / (6378137.0 * math.cos(math.radians(latitude)))),
            point.up)


def from_gps(latitude, longitude, altitude, origin=(-35.363261, 149.165230)):
    return Point(math.radians(latitude - origin[0]) * 6378137.0,
                 math.radians(longitude - origin[1]) * 6378137.0 * math.cos(math.radians(origin[0])),
                 altitude)


@dataclass(frozen=True)
class Obstacle:
    north: float
    east: float
    radius: float
    height: float

    def intersects(self, a, b, clearance=2.0):
        # Conservative cylindrical exclusion for the entire segment, including return.
        if min(a.up, b.up) > self.height + clearance:
            return False
        dn, de = b.north - a.north, b.east - a.east
        length2 = dn * dn + de * de
        t = max(0., min(1., ((self.north-a.north)*dn + (self.east-a.east)*de) / length2)) if length2 else 0.
        return math.hypot(a.north+t*dn-self.north, a.east+t*de-self.east) <= self.radius+clearance


@dataclass(frozen=True)
class Plan:
    points: tuple[Point, ...]
    speed: float
    destination: str
    loaded_roundtrip_wh: float


class State(str, Enum):
    IDLE = "IDLE"
    VALIDATED = "VALIDATED"
    OUTBOUND = "OUTBOUND"
    AT_PAD = "AT_PAD"
    DELIVERY = "DELIVERY"
    RETURNING = "RETURNING"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"
    HOLD = "HOLD"


class Authenticator:
    """Session-bound HMAC + strict sequence + TTL; no persisted key or remote release verb.

    Session resets on restart; in-flight missions cannot be resumed through this API.
    The local trusted operator must obtain the new session and hold the ephemeral key.
    """
    def __init__(self, key=None):
        self.key = key or secrets.token_bytes(32)
        if len(self.key) < 32:
            raise ValueError("key too short")
        self.session = secrets.token_hex(16)
        self.sequence = -1

    @staticmethod
    def canonical(body):
        try:
            return json.dumps(body, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        except (ValueError, TypeError) as e:
            raise Rejected("invalid JSON values") from e

    def sign(self, payload, now, sequence=0):
        body = dict(session=self.session, sequence=sequence, issued=now, payload=payload)
        return dict(body=body, mac=hmac.new(self.key, self.canonical(body), hashlib.sha256).hexdigest())

    def verify(self, envelope, now):
        if not isinstance(envelope, dict) or set(envelope) != {"body", "mac"}:
            raise Rejected("invalid envelope")
        body, mac = envelope["body"], envelope["mac"]
        if not isinstance(body, dict) or set(body) != {"session", "sequence", "issued", "payload"}:
            raise Rejected("invalid body")
        if not isinstance(mac, str) or not hmac.compare_digest(mac, hmac.new(self.key, self.canonical(body), hashlib.sha256).hexdigest()):
            raise Rejected("unauthenticated")
        if body["session"] != self.session:
            raise Rejected("wrong session")
        seq = body["sequence"]
        if type(seq) is not int or seq <= self.sequence:
            raise Rejected("replayed sequence")
        if not 0 <= number(now) - number(body["issued"]) <= 5.0:
            raise Rejected("stale or future command")
        self.sequence = seq  # consume authenticated requests even if policy rejects them
        return body["payload"]


@dataclass(frozen=True)
class Health:
    timestamp: float
    battery_wh: float = 180.
    voltage: float = 24.
    gnss_ok: bool = True
    telemetry_ok: bool = True
    rc_ok: bool = True
    companion_ok: bool = True
    estimator_ok: bool = True

    def fresh(self, now):
        try:
            return (all(type(v) is bool for v in (self.gnss_ok,self.telemetry_ok,self.rc_ok,self.companion_ok,self.estimator_ok))
                    and 0 <= number(now)-number(self.timestamp) <= 2 and number(self.battery_wh) >= 0 and number(self.voltage) > 0)
        except Rejected:
            return False


def energy_wh(distance, speed, loaded=True):
    """Sizing proxy only: 900/650 W loaded/empty plus 30 s contingency, 20% reserve.

    Not calibrated to MN4110/P16 data, not used to assert aircraft endurance.
    """
    distance, speed = number(distance), number(speed)
    if distance < 0 or speed <= 0:
        raise Rejected("invalid energy-model distance/speed")
    return (900. if loaded else 650.) * (distance/speed + 30.) / 3600. / 0.8


class Coordinator:
    def __init__(self, auth=None, obstacles=()):
        self.auth = auth or Authenticator()
        self.obstacles = tuple(obstacles)
        self.state = State.IDLE
        self.events = []
        self.plan = None
        self.payload = "retained"

    def transition(self, state, reason):
        self.events.append(dict(event="mission_state", previous=self.state.value, state=state.value, reason=reason))
        self.state = state

    def submit(self, envelope, health, now, *, landed, armed):
        try:
            request = self.auth.verify(envelope, now)
            if self.state != State.IDLE or landed is not True or armed is not False:
                raise Rejected("mission requires idle, landed, disarmed vehicle")
            if not health.fresh(now) or not all((health.gnss_ok, health.telemetry_ok, health.rc_ok, health.companion_ok, health.estimator_ok)):
                raise Rejected("unhealthy vehicle")
            if not isinstance(request, dict) or set(request) != {"destination", "waypoints", "speed"}:
                raise Rejected("unsupported request fields")
            if request["destination"] != "delivery-pad":
                raise Rejected("unknown destination")
            speed = number(request["speed"])
            if not 0.5 <= speed <= 5:
                raise Rejected("speed outside envelope")
            if not isinstance(request["waypoints"], list) or not 1 <= len(request["waypoints"]) <= 20:
                raise Rejected("invalid waypoint count")
            points = tuple(Point.parse(p) for p in request["waypoints"])
            if any(math.hypot(p.north, p.east) > 80 or not 3 <= p.up <= 15 for p in points):
                raise Rejected("route outside fence/altitude envelope")
            if math.hypot(points[-1].north-20, points[-1].east-10) > 0.25:
                raise Rejected("destination coordinates do not match registered pad")
            route = (Point(0, 0, points[0].up),) + points
            # Include vertical takeoff/landing segments and direct emergency RTL corridor.
            segments = list(zip(route, route[1:])) + [(Point(0,0,0), route[0]), (points[-1], Point(20,10,0))]
            segments += [(p, Point(0,0,p.up)) for p in points]
            if any(o.intersects(a,b) for a,b in segments for o in self.obstacles):
                raise Rejected("route or fallback corridor blocked")
            distance = 2 * sum(a.distance(b) for a,b in zip(route, route[1:])) + 4*max(p.up for p in points)
            budget = energy_wh(distance, speed)
            if health.voltage < 22.2 or health.battery_wh < budget:
                raise Rejected("insufficient loaded roundtrip energy reserve")
            self.plan = Plan(points, speed, "delivery-pad", budget)
            self.transition(State.VALIDATED, "authenticated route and loaded return reserve accepted")
            return self.plan
        except Rejected as e:
            self.events.append(dict(event="request_rejected", reason=str(e)))
            raise

    def submit_navigation(self, envelope, health, now, *, landed, armed):
        """Validate a local-position SITL round trip; no delivery or hardware action."""
        try:
            request = self.auth.verify(envelope, now)
            if self.state != State.IDLE or landed is not True or armed is not False:
                raise Rejected("mission requires idle, landed, disarmed vehicle")
            if not health.fresh(now) or not all((health.gnss_ok, health.telemetry_ok,
                                                health.rc_ok, health.companion_ok, health.estimator_ok)):
                raise Rejected("unhealthy vehicle")
            if not isinstance(request, dict) or set(request) != {"position", "speed"}:
                raise Rejected("unsupported request fields")
            target = Point.parse(request["position"])
            speed = number(request["speed"])
            if not .5 <= speed <= 5:
                raise Rejected("speed outside envelope")
            if math.hypot(target.north, target.east) > 80 or not 3 <= target.up <= 15:
                raise Rejected("position outside fence/altitude envelope")
            if math.hypot(target.north, target.east) < 3:
                raise Rejected("position too close to launch for a navigation leg")
            home = Point(0, 0, target.up)
            segments = [(Point(0,0,0), home), (home, target),
                        (target, home), (home, Point(0,0,0))]
            if any(o.intersects(a, b) for a, b in segments for o in self.obstacles):
                raise Rejected("route or return corridor blocked")
            distance = 2*home.distance(target) + 2*target.up
            budget = energy_wh(distance, speed)
            if health.voltage < 22.2 or health.battery_wh < budget:
                raise Rejected("insufficient loaded roundtrip energy reserve")
            self.plan = Plan((target, home), speed, "position", budget)
            self.transition(State.VALIDATED, "authenticated position and loaded return reserve accepted")
            return self.plan
        except Rejected as e:
            self.events.append(dict(event="request_rejected", reason=str(e)))
            raise

    def airborne(self):
        if self.state != State.VALIDATED:
            raise Rejected("invalid takeoff phase")
        self.transition(State.OUTBOUND, "flight controller reports airborne")

    def at_pad(self, distance, landed):
        if self.state != State.OUTBOUND or not landed or not 0 <= number(distance) <= 1.5:
            raise Rejected("pad arrival not established")
        self.transition(State.AT_PAD, "landed at registered destination")

    def begin_delivery(self):
        if self.state != State.AT_PAD:
            raise Rejected("delivery phase not authorized")
        self.transition(State.DELIVERY, "evaluating local interlocks")

    def approach(self, health, now, *, destination_valid, pad_clear, target_valid, range_valid):
        if self.state != State.OUTBOUND:
            raise Rejected("approach requires outbound mission")
        if not health.fresh(now) or not health.gnss_ok or not health.estimator_ok:
            self.transition(State.ABORTED, "approach navigation invalid; request LAND")
            return "LAND"
        if any(value is not True for value in (destination_valid,pad_clear,target_valid,range_valid)):
            self.transition(State.ABORTED, "destination/approach evidence invalid; retain payload and request RTL")
            return "RTL"
        return None

    def return_leg(self, payload, distance, health, now):
        if self.state not in (State.AT_PAD, State.DELIVERY):
            raise Rejected("invalid return phase")
        if payload not in ("retained", "separated"):
            self.payload = "unknown"
            self.transition(State.HOLD, "latch/payload uncertain; no takeoff")
            return None
        self.payload = payload
        budget = energy_wh(distance, self.plan.speed, loaded=payload != "separated")
        if not health.fresh(now) or not all((health.gnss_ok,health.telemetry_ok,health.rc_ok,health.companion_ok,health.estimator_ok)) or health.voltage < 22.2 or health.battery_wh < budget:
            self.transition(State.HOLD, "return health/energy reserve inadequate; no takeoff")
            return None
        self.transition(State.RETURNING, f"{payload} payload; return sizing {budget:.3f} Wh")
        return budget

    def finish(self, home_distance, landed, armed):
        if self.state != State.RETURNING or not landed or armed or not 0 <= number(home_distance) <= 2:
            raise Rejected("home landing not established")
        self.transition(State.COMPLETE, "home, landed, disarmed")

    def monitor(self, health, now, return_distance=30.):
        if self.state not in (State.OUTBOUND, State.RETURNING):
            return None
        if not health.fresh(now) or not health.gnss_ok or not health.estimator_ok or health.voltage < 21.0:
            action = "LAND"
        elif not all((health.telemetry_ok, health.rc_ok, health.companion_ok)) or health.battery_wh < energy_wh(return_distance, self.plan.speed, self.payload != "separated") or health.voltage < 22.2:
            action = "RTL"
        else:
            return None
        self.transition(State.ABORTED, f"health fault; request {action}; FC remains authoritative")
        return action
