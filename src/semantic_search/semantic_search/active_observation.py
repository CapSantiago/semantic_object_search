"""Bounded next-view selection from occupancy and measured RGB-D visibility.

The utility is unseen geometric samples / travel penalty. It is a visibility
heuristic, not a learned semantic policy or evidence that no objects remain.
"""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt, label
from scipy.spatial.transform import Rotation

from .cleaning import RoomSweep
from .visibility import visible_samples


class ActiveObservation:
    def __init__(self, data, width, height, resolution, origin, rooms, room, start,
                 *, max_views=6, target_fraction=.85, spacing=.35):
        self.sweep = RoomSweep(data, width, height, resolution, origin, rooms, room, start)
        if not isinstance(max_views,int) or not 1<=max_views<=12 or not 0<target_fraction<=1:
            raise ValueError('Invalid active observation budget or target')
        stride = max(1, round(spacing/resolution))
        rows, cols = np.nonzero(self.sweep.floor)
        chosen = (rows % stride == stride//2) & (cols % stride == stride//2)
        xy = [self.sweep.point(c, r) for r, c in zip(rows[chosen], cols[chosen])]
        self.points = np.array([(x,y,z) for x,y in xy for z in (.15,.45,.85,1.25)]).reshape(-1,3)
        self.seen = np.zeros(len(self.points), dtype=bool)
        self.full_centers = distance_transform_edt(np.pad(self.sweep.free,1))[1:-1,1:-1]*resolution >= .35
        self.max_views, self.target_fraction = max_views, target_fraction
        self.camera = None
        self.frames = self.selected = 0
        self.reason = 'awaiting_rgbd'
        self.decisions = []
        self.focused_views = 0
        self.target_rechecks = self.surface_inspections = 0

    def observe(self, depth, k, xyz, xyzw, base_pose, *, scale=1.):
        if not len(self.points):
            return
        self.seen |= visible_samples(self.points,depth,k,xyz,xyzw,depth_scale=scale)
        self.camera = (np.asarray(k),np.asarray(xyz),Rotation.from_quat(xyzw),np.asarray(base_pose),np.shape(depth))
        self.frames += 1

    def _line_of_sight(self, start, points):
        """Conservative 2-D first-obstacle prediction; actual updates use RGB-D."""
        if not len(points):
            return np.zeros(0,dtype=bool)
        # Check intermediate cells; the endpoint may lie on a first surface.
        fractions = np.arange(1,81)/81
        xy = np.asarray(start) + (points[:,:2]-start)[:,None,:]*fractions[None,:,None]
        dx,dy = xy[:,:,0]-self.sweep.origin[0],xy[:,:,1]-self.sweep.origin[1]
        col = np.floor((self.sweep.cosine*dx+self.sweep.sine*dy)/self.sweep.resolution).astype(int)
        row = np.floor((-self.sweep.sine*dx+self.sweep.cosine*dy)/self.sweep.resolution).astype(int)
        h,w = self.sweep.free.shape
        inside = (row>=0)&(row<h)&(col>=0)&(col<w)
        return (inside & self.sweep.free[np.clip(row,0,h-1),np.clip(col,0,w-1)]).all(axis=1)

    def next_view(self, current, attempted, available, tentative=(), supports=()):
        if self.camera is None:
            self.reason = 'no_measured_camera'
            return None
        if self.selected >= self.max_views:
            self.reason = 'view_budget_exhausted'
            return None
        targets = [item for item in tentative if len(item)==4 and np.isfinite(item).all()]
        source = 'tentative_object_recheck'
        if not targets:
            targets = [item for item in supports if len(item)==4 and np.isfinite(item).all()]
            source = 'measured_support_surface'
        focus = bool(targets) and self.focused_views<2
        if not len(self.seen) or (self.seen.mean() >= self.target_fraction and not focus):
            self.reason = 'sample_target_reached'
            return None
        available = np.asarray(available,dtype=bool)
        if available.shape != self.sweep.free.shape:
            raise ValueError('Active observation costmap does not match map')
        connected, _ = label(self.full_centers & available)
        col,row = self.sweep.index(*current[:2])
        h,w = connected.shape
        if not (0<=row<h and 0<=col<w) or connected[row,col]==0:
            self.reason = 'current_pose_not_in_safe_component'
            return None
        safe = self.sweep.centers & (connected==connected[row,col])
        step = max(1,round(.70/self.sweep.resolution))
        rows,cols = np.nonzero(safe)
        select = (rows % step == step//2)&(cols % step == step//2)
        positions = [self.sweep.point(c,r) for r,c in zip(rows[select],cols[select])]
        # Keep evaluation bounded in large rooms; deterministic spatial sampling.
        positions = positions[::max(1,math.ceil(len(positions)/120))]
        k, xyz, camera_rotation, base, image_shape = self.camera
        predicted_depth = np.full(image_shape,4.)
        if focus:
            best_focus = None
            for identity,tx,ty,tz in targets:
                target = np.array([[tx,ty,tz]])
                for x,y in positions:
                    distance = math.hypot(tx-x,ty-y)
                    if not .8<=distance<=2.2:
                        continue
                    heading = math.atan2(ty-y,tx-x)
                    pose = (x,y,heading)
                    if any(math.dist(pose[:2],p[:2])<.45 and
                           abs(math.remainder(heading-p[2],2*math.pi))<.5 for p in attempted):
                        continue
                    delta = Rotation.from_euler('z',heading-base[2])
                    translation = delta.apply(xyz-np.array([base[0],base[1],0]))+np.array([x,y,0])
                    orientation = delta*camera_rotation
                    optical = orientation.inv().apply(target-translation)[0]
                    if optical[2]<=.1:
                        continue
                    u,v = k[0]*optical[0]/optical[2]+k[2],k[4]*optical[1]/optical[2]+k[5]
                    h,w = image_shape
                    if not (.12*w<u<.88*w and .12*h<v<.88*h):
                        continue
                    turn = abs(math.remainder(heading-current[2],2*math.pi))
                    utility = 1/(distance*(1+.25*math.dist(current[:2],(x,y))+.10*turn))
                    if best_focus is None or utility>best_focus[0]:
                        best_focus = (utility,pose,int(identity),[tx,ty,tz])
            if best_focus is not None:
                self.selected += 1
                self.focused_views += 1
                self.target_rechecks += source=='tentative_object_recheck'
                self.surface_inspections += source=='measured_support_surface'
                self.reason = source
                self.decisions.append(dict(source=source,pose=list(best_focus[1]),
                    tentative_id=best_focus[2] if source=='tentative_object_recheck' else None,
                    measured_hint_id=best_focus[2],measured_hint_position=best_focus[3],utility=best_focus[0],
                    observed_samples=int(self.seen.sum()),predicted_new_samples=None))
                return best_focus[1]
        if self.seen.mean()>=self.target_fraction:
            self.reason = 'sample_target_reached'
            return None
        unseen = self.points[~self.seen]
        best = None
        for x,y in positions:
            local = (unseen[:,0]-x)**2+(unseen[:,1]-y)**2 <= 4.**2
            subset = unseen[local]
            los = self._line_of_sight((x,y),subset)
            subset = subset[los]
            if len(subset)<4:
                continue
            for heading in np.arange(-math.pi,math.pi,math.pi/4):
                pose = (x,y,float(heading))
                if any(math.dist(pose[:2],p[:2])<.45 and abs(math.remainder(heading-p[2],2*math.pi))<.5
                       for p in attempted):
                    continue
                delta = Rotation.from_euler('z',heading-base[2])
                translation = delta.apply(xyz-np.array([base[0],base[1],0]))+np.array([x,y,0])
                orientation = (delta*camera_rotation).as_quat()
                # Synthetic range is ONLY a prediction used to rank safe views.
                predicted = visible_samples(subset,predicted_depth,
                                            k,translation,orientation,surface_tolerance=0.)
                gain = int(predicted.sum())
                turn = abs(math.remainder(heading-current[2],2*math.pi))
                utility = gain/(1+.35*math.dist(current[:2],(x,y))+.10*turn)
                if gain>=4 and (best is None or utility>best[0]):
                    best = (utility,pose,gain)
        if best is None:
            self.reason = 'no_safe_view_with_predicted_gain'
            return None
        self.selected += 1
        self.reason = 'supplementing'
        self.decisions.append(dict(source='active_visibility',pose=list(best[1]),predicted_new_samples=best[2],utility=best[0],
                                   observed_samples=int(self.seen.sum())))
        return best[1]

    def summary(self):
        before = self.decisions[0]['observed_samples'] if self.decisions else int(self.seen.sum())
        return dict(method='measured_rgbd_visibility_and_bounded_next_view',
                    sample_heights_m=[.15,.45,.85,1.25],samples=len(self.seen),
                    observed_samples=int(self.seen.sum()),
                    observed_fraction=float(self.seen.mean()) if len(self.seen) else 0.,
                    observed_before_supplement=before,new_samples_from_supplement=int(self.seen.sum())-before,
                    target_fraction=self.target_fraction,additional_views=self.selected,
                    tentative_recheck_views=self.target_rechecks,support_surface_views=self.surface_inspections,
                    maximum_semantic_views=2,maximum_tentative_recheck_views=2,
                    maximum_additional_views=self.max_views,frames=self.frames,stop_reason=self.reason,
                    decisions=self.decisions,full_room_coverage_verified=False,
                    limitations=['Known free grid samples at four heights; furniture interiors are not evaluated.',
                                 'Geometric visibility does not prove recognition or absence.',
                                 'Tentative-target frustum predictions require actual depth and recognition after arrival.',
                                 '2-D occlusion prediction is conservative; coverage updates use measured depth only.'])

    def evidence(self):
        return dict(frame_id='map',points=self.points.round(3).tolist(),observed=self.seen.tolist(),**self.summary())
