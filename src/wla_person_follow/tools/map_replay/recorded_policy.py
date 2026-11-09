"""TrackingPolicy as recorded by rec_planning.py: the fields the MPC reads and
its two predicates frozen at policy_time (both are pure in follow_session)."""


class RecordedPolicy:
    def __init__(self, policy, now):
        self.search_arrival_m = policy.search_arrival_m
        self.search_heading = policy.search_heading
        self.search_started = policy.search_started
        self.search_side = policy.search_side
        self.now = now
        self.allowed = bool(policy.translation_allowed(now))
        self.searching = bool(policy.active_search(now))

    def translation_allowed(self, now):
        assert now == self.now
        return self.allowed

    def active_search(self, now):
        assert now == self.now
        return self.searching
