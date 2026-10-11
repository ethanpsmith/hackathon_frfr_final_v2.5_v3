"""Geographic navigation policy for the Columbia simulation study.

Coordinates are WGS84. This profile has no terrain, obstacle survey or landing-pad
evidence, so it authorizes navigation/return in simulation, never payload release.
"""
from dataclasses import dataclass
import math
from geographiclib.geodesic import Geodesic
from .core import Coordinator, Plan, Point, Rejected, State, energy_wh, number

GEOD = Geodesic.WGS84


def coordinate(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise Rejected("coordinate must be [latitude, longitude]")
    lat, lon = (number(v) for v in value)
    if not 33.97 <= lat <= 34.04 or not -81.08 <= lon <= -81.00:
        raise Rejected("coordinate outside Columbia simulation study area")
    return lat, lon


def distance(a, b):
    return GEOD.Inverse(*a, *b)["s12"]


@dataclass(frozen=True)
class GeographicRoute:
    path: tuple
    altitude: float
    speed: float

    @classmethod
    def parse(cls, request):
        if not isinstance(request, dict) or set(request) != {"path", "altitude", "speed"}:
            raise Rejected("expected path, altitude, speed")
        if not isinstance(request["path"], list) or not 2 <= len(request["path"]) <= 20:
            raise Rejected("route requires 2–20 geographic coordinates")
        path = tuple(coordinate(p) for p in request["path"])
        altitude, speed = number(request["altitude"]), number(request["speed"])
        if not 20 <= altitude <= 80 or not .5 <= speed <= 5:
            raise Rejected("geographic SITL profile requires altitude 20–80 m and speed 0.5–5 m/s")
        legs = [distance(a,b) for a,b in zip(path,path[1:])]
        if any(d < 3 for d in legs) or sum(legs) > 2000:
            raise Rejected("route legs must exceed 3 m; outbound path must not exceed 2 km")
        return cls(path, altitude, speed)

    @property
    def outbound_m(self):
        return sum(distance(a,b) for a,b in zip(self.path,self.path[1:]))

    @property
    def fence_radius(self):
        return math.ceil(max(distance(self.path[0],p) for p in self.path)/50)*50+100

    @property
    def budget_wh(self):
        return energy_wh(2*self.outbound_m+2*self.altitude, self.speed)

    def local(self, lat, lon, altitude):
        g = GEOD.Inverse(*self.path[0], lat, lon)
        bearing = math.radians(g["azi1"])
        return Point(g["s12"]*math.cos(bearing), g["s12"]*math.sin(bearing), altitude)

    def global_position(self, point):
        g = GEOD.Direct(*self.path[0], math.degrees(math.atan2(point.east,point.north)),
                        math.hypot(point.north,point.east))
        return g["lat2"], g["lon2"], point.up

    def summary(self):
        return dict(path=self.path, altitude=self.altitude, speed=self.speed,
                    outbound_m=self.outbound_m, roundtrip_m=2*self.outbound_m,
                    fence_radius_m=self.fence_radius, loaded_roundtrip_estimate_wh=self.budget_wh,
                    home_msl_assumption_m=100, hardware=False,
                    coordinate_source="Flightpath presentation inputs or operator coordinates; unsurveyed",
                    model="stock ArduPilot X quad on flat ground; no city obstacles or terrain")


class GeographicCoordinator(Coordinator):
    def submit_geographic(self, envelope, health, now, *, landed, armed, expected_route):
        try:
            route = GeographicRoute.parse(self.auth.verify(envelope, now))
            if route != expected_route:
                raise Rejected("route differs from owned simulator home/fence configuration")
            if self.state != State.IDLE or landed is not True or armed is not False:
                raise Rejected("mission requires idle, landed, disarmed vehicle")
            if not health.fresh(now) or not all((health.gnss_ok, health.telemetry_ok, health.rc_ok,
                                               health.companion_ok, health.estimator_ok)):
                raise Rejected("unhealthy vehicle")
            if health.voltage < 22.2 or health.battery_wh < route.budget_wh:
                raise Rejected("insufficient loaded roundtrip energy reserve")
            # Reverse the entire route; every intermediate waypoint is preserved.
            points = route.path[1:] + tuple(reversed(route.path[:-1]))
            self.plan = Plan(tuple(route.local(*p,route.altitude) for p in points),
                             route.speed, "geographic-navigation", route.budget_wh)
            self.transition(State.VALIDATED, "WGS84 route, simulator fence and loaded return reserve accepted")
            return self.plan
        except Rejected as error:
            self.events.append(dict(event="request_rejected", reason=str(error)))
            raise
