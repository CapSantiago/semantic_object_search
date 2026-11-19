import numpy as np
import pytest
from semantic_search.visibility import visible_samples


def test_frustum_occlusion_and_missing_depth():
    depth=np.full((5,5),2.)
    depth[2,3]=np.nan
    k=[2,0,2,0,2,2,0,0,1]
    points=[[0,0,1],[0,0,3],[0,0,-1],[10,0,1],[.5,0,1],[0,0,2.04]]
    assert visible_samples(points,depth,k,[0,0,0],[0,0,0,1]).tolist()==[True,False,False,False,False,True]


def test_camera_translation_rotation_and_millimetres():
    # Camera optical z points along map x, optical x along negative map z.
    q=[0,np.sin(np.pi/4),0,np.cos(np.pi/4)]
    got=visible_samples([[2,2,3],[4,2,3],[0,2,3]],np.full((5,5),2000),
                        [2,0,2,0,2,2,0,0,1],[1,2,3],q,depth_scale=.001)
    assert got.tolist()==[True,False,False]
    with pytest.raises(ValueError):
        visible_samples([[0,0,1]],np.ones((5,5)),np.eye(3),[0,0,0],[0,0,0,0])
