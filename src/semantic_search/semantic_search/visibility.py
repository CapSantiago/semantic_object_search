"""Offline visibility evaluation from measured optical-frame RGB-D, without truth."""
import numpy as np


def visible_samples(points, depth, intrinsics, translation, quaternion, *,
                    depth_scale=1., max_range=4., surface_tolerance=.05):
    """Test map samples against the measured camera frustum and first surface.

    True means geometric line of sight at this sample, not object recognition.
    Missing depth is unknown (False). Camera uses optical x-right/y-down/z-forward.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12:
        raise ValueError('Expected a finite nonzero xyzw quaternion')
    x,y,z,w = q / np.linalg.norm(q)
    rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    camera = (points-np.asarray(translation)) @ rotation
    k = np.asarray(intrinsics).reshape(3,3)
    depth = np.asarray(depth)*depth_scale
    valid = np.all(np.isfinite(camera),axis=1) & (camera[:,2]>.2) & (camera[:,2]<=max_range)
    indices = np.flatnonzero(valid)
    valid[:] = False
    pixels = camera[indices] @ k.T
    uv = np.rint(pixels[:,:2]/pixels[:,2:]).astype(int)
    inside = (uv[:,0]>=0)&(uv[:,0]<depth.shape[1])&(uv[:,1]>=0)&(uv[:,1]<depth.shape[0])
    indices,uv = indices[inside],uv[inside]
    measured = depth[uv[:,1],uv[:,0]]
    valid[indices] = np.isfinite(measured)&(measured>0)&(camera[indices,2]<=measured+surface_tolerance)
    return valid
