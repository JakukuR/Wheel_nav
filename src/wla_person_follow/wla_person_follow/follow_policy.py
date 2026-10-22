"""Immutable geometry policy shared by candidate generation and output."""
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class PolicySnapshot:
    state: str
    last_seen: float = None
    lost_since: float = None
    grace_limit_s: float = .50
    active_search_s: float = 8.
    target_id: str = None

    @classmethod
    def from_binding(cls,binding,now):
        return cls(state=binding.state,last_seen=binding.last_seen,lost_since=binding.lost_since,
            grace_limit_s=binding.c.freshness_s+binding.c.missing_grace_s,
            active_search_s=binding.c.active_search_s,target_id=binding.resolved_id or None)

    def effective_state(self,now):
        if self.state in ('lost','ended','suspect','binding'):return self.state
        if self.last_seen is None or now-self.last_seen>self.grace_limit_s:return 'lost'
        return self.state

    def translation_allowed(self,now):
        return self.effective_state(now) in ('bound_tracking','bound_untracked','bound_coasting')

    def active_search(self,now):
        start=self.lost_since
        if start is None and self.last_seen is not None:start=self.last_seen+self.grace_limit_s
        return self.effective_state(now)=='lost' and start is not None and 0<=now-start<self.active_search_s

    def limit_command(self,command,now):
        result=np.array(command,dtype=float,copy=True)
        if not self.translation_allowed(now):
            result[0]=0.
            if not self.active_search(now):result[1]=0.
        return result
