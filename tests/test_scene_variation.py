import xml.etree.ElementTree as ET

import pytest

from semantic_search.development import apply_scene_model_poses


def test_independent_scene_pose_variation_keeps_original_names_and_validates_input():
    world=ET.fromstring('<world><model name="Ball_01_001"><pose>1 2 .31 0 0 0</pose></model><model name="turtlebot3_waffle"/></world>')
    original=ET.tostring(world)
    assert apply_scene_model_poses(world,{})=={} and ET.tostring(world)==original
    pose=[2.,3.,.31,0,0,0]
    assert apply_scene_model_poses(world,{'Ball_01_001':pose})=={'Ball_01_001':pose}
    assert world.find("model[@name='Ball_01_001']/pose").text=='2.0 3.0 0.31 0 0 0'
    for change in ({'missing':pose},{'turtlebot3_waffle':pose},{'Ball_01_001':[0]*5},
                   {'Ball_01_001':[0,0,float('nan'),0,0,0]},[]):
        with pytest.raises(ValueError):apply_scene_model_poses(world,change)
