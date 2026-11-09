"""Explicit scene identities and separate assets for the baseline and AWS house."""
import json
from pathlib import Path


def load_scene(scene_id, project_root, config_dir=None):
    if config_dir is None:
        from ament_index_python.packages import get_package_share_directory
        config_dir = Path(get_package_share_directory('semantic_search')) / 'config'
    registry = json.loads((Path(config_dir) / 'scenes.json').read_text(encoding='utf-8'))
    if scene_id not in registry['scenes']:
        raise ValueError(f'Unknown scene {scene_id!r}; choose: {", ".join(registry["scenes"])}')
    scene = dict(registry['scenes'][scene_id], id=scene_id)
    root = Path(project_root).absolute()
    for key in ('map', 'rooms', 'route', 'output'):
        scene[key] = str(root / scene[key])
    return scene


if __name__ == '__main__':
    import sys
    scene = load_scene(sys.argv[1], sys.argv[2], Path(__file__).parents[1] / 'config')
    for key in ('label', 'world', 'map', 'rooms', 'route', 'output'):
        print(scene[key])
