import numpy as np
from semantic_search.ball_geometry import BALL_SUBTYPES, sphere_center


def test_partial_sphere_recovers_center_under_rotation_and_depth_noise():
    rng=np.random.default_rng(42)
    vectors=rng.normal(size=(4000,3));vectors/=np.linalg.norm(vectors,axis=1)[:,None]
    vectors=vectors[vectors[:,0]<-.2]
    center=np.array([-5.7,2.3,.24])
    points=center+.24*vectors+rng.normal(0,.001,(len(vectors),3))
    actual=sphere_center(points)
    assert actual is not None and np.linalg.norm(np.asarray(actual)-center)<.01
    # A visible-surface median would be strongly biased towards the camera.
    assert np.linalg.norm(np.median(points,axis=0)-center)>.10


def test_planar_background_and_insufficient_or_invalid_geometry_do_not_fit():
    rng=np.random.default_rng(2);points=rng.uniform(-.4,.4,(800,3));points[:,2]=.7
    assert sphere_center(points) is None
    assert sphere_center(points[:10]) is None
    assert sphere_center([[np.nan,0,1]]*100) is None
    assert 'balloon' not in BALL_SUBTYPES
