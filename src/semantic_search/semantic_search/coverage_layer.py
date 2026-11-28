"""Compact display of measured brush cells; never used by the planner."""
import hashlib
import numpy as np


def encode_runs(mask):
    flat=np.asarray(mask,dtype=bool).ravel()
    edges=np.flatnonzero(np.diff(np.r_[False,flat,False])).reshape(-1,2)
    return [[int(a),int(b-a)]for a,b in edges]


class CoverageDisplay:
    def __init__(self,sweep):
        h,w=sweep.occupancy.shape
        self.eligible=sweep.eligible.copy()
        self.base=dict(frame_id='map',width=w,height=h,resolution=sweep.resolution,
            origin=dict(zip(('x','y','yaw'),sweep.origin)),
            data_sha256=hashlib.sha256(sweep.occupancy.astype(np.int8).tobytes()).hexdigest(),
            eligible_runs=encode_runs(self.eligible),eligible_cells=int(self.eligible.sum()))

    def snapshot(self,covered,*,stamp_seconds,blocked=None,cost_stamp_seconds=None):
        covered=np.asarray(covered,dtype=bool)
        if covered.shape!=self.eligible.shape or np.any(covered&~self.eligible):
            raise ValueError('实际覆盖掩膜与底图不一致。')
        if blocked is not None and np.shape(blocked)!=covered.shape:
            raise ValueError('代价图与实际覆盖尺寸不一致。')
        return dict(self.base,stamp_seconds=stamp_seconds,
            covered_runs=encode_runs(covered),covered_cells=int(covered.sum()),
            blocked_runs=encode_runs(np.asarray(blocked,dtype=bool)&self.eligible)if blocked is not None else [],
            cost_stamp_seconds=cost_stamp_seconds,blocked_observation_available=blocked is not None)
