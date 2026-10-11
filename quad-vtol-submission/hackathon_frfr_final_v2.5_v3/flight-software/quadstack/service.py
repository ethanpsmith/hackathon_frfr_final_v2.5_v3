"""Transport-independent companion API. No arming, servo, or parameter API."""
from typing import Protocol
from .core import Coordinator, State


class FlightController(Protocol):
    def upload(self, plan): ...
    def request_recovery(self, mode): ...


class CompanionService:
    def __init__(self, controller: FlightController, coordinator=None):
        self.controller = controller
        self.coordinator = coordinator or Coordinator()

    def request(self, envelope, health, now, *, landed, armed):
        plan = self.coordinator.submit(envelope, health, now, landed=landed, armed=armed)
        try:
            self.controller.upload(plan)
        except Exception:
            self.coordinator.transition(State.ABORTED, "mission upload failed; no flight authorized")
            raise
        return plan

    def request_navigation(self, envelope, health, now, *, landed, armed):
        plan = self.coordinator.submit_navigation(envelope, health, now, landed=landed, armed=armed)
        try:
            self.controller.upload(plan)
        except Exception:
            self.coordinator.transition(State.ABORTED, "navigation upload failed; no flight authorized")
            raise
        return plan

    def tick(self, health, now, return_distance):
        mode = self.coordinator.monitor(health, now, return_distance)
        if mode:
            self.controller.request_recovery(mode)
        return mode
