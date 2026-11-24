"""Sensor-only sphere centre fit for segmented balls; no fixture radius/position."""
import numpy as np
from scipy.optimize import least_squares

# Explicit physical subtypes only. A balloon is deliberately not a sports ball.
BALL_SUBTYPES=frozenset(('sports ball','exercise ball','ball','beach ball','soccer ball',
                        'basketball','volleyball','tennis ball','football'))


def sphere_center(points):
    """Fit a curved visible surface, rejecting planes and unreliable depth masks.

    The numerical solver is SciPy's robust least_squares. Bounds are broad
    class-size priors, unrelated to a scene asset. Return None on weak geometry.
    """
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3:return None
    points=points[np.isfinite(points).all(axis=1)]
    if len(points)<100:return None
    points=points[::max(1,len(points)//1500)]
    origin=np.median(points,axis=0);local=points-origin
    singular=np.linalg.svd(local-local.mean(axis=0),compute_uv=False)
    if singular[-1]<.02*singular[0]:return None
    a=np.column_stack((2*local,np.ones(len(local))))
    fit,_,rank,_=np.linalg.lstsq(a,np.sum(local*local,axis=1),rcond=None)
    if rank!=4:return None
    radius=np.sqrt(max(0,fit[3]+fit[:3]@fit[:3]))
    if not .025<radius<.65:return None
    solution=least_squares(lambda v:np.linalg.norm(local-v[:3],axis=1)-v[3],
        np.r_[fit[:3],radius],loss='soft_l1',f_scale=.005,max_nfev=80)
    if not solution.success or not .025<solution.x[3]<.65:return None
    residual=np.abs(solution.fun)
    if np.quantile(residual,.8)>.015 or np.mean(residual<.025)<.9:return None
    center=origin+solution.x[:3]
    if not np.isfinite(center).all():return None
    return tuple(float(v) for v in center)
