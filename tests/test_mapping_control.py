from math import pi
from types import SimpleNamespace

import numpy as np
import pytest

from semantic_search.mapping_control import obstacle, parse_motion, short_detour


def make_grid(free=True):
    origin = SimpleNamespace(position=SimpleNamespace(x=-2.,y=-2.),
                            orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
    data = np.full((80,80),0 if free else -1)
    info = SimpleNamespace(width=80,height=80,resolution=.05,origin=origin)
    return data,SimpleNamespace(info=info,data=data.reshape(-1))


def test_motion_language_does_not_execute_negative_or_combined_commands():
    assert parse_motion('请向前走。')=='forward'
    assert parse_motion('请左转一下')=='left'
    for command in ('不要前进','前进后退','前进到厨房','前进或左转','快一点然后停下'):
        with pytest.raises(ValueError):
            parse_motion(command)


def test_swept_area_depends_on_motion_direction():
    points=[(-.34,0),(.10,.50)]
    assert obstacle(points,.12,0) is None
    assert obstacle(points,-.12,0)==.34
    assert obstacle([(.04,.26)],0,.55) is not None
    assert obstacle([(.04,.26)],0,-.55) is not None


def test_detour_checks_current_scan_and_requires_known_space():
    _,grid=make_grid()
    route=short_detour(grid,(0.,0.),0.,[(.39,0.)])
    assert route and route[-1]==(1.,0.) and abs(route[0][1])>=.60
    assert short_detour(grid,(0.,0.),0.) is None
    data,blocked=make_grid()
    data[:,47:51] = -1
    assert short_detour(blocked,(0.,0.),0.,[(.39,0.)]) is None
    data,turned=make_grid()
    data[48:52,39:41]=100
    assert short_detour(turned,(0.,0.),pi/2)[-1][1]>=1.
