"""One file-map selection boundary; live /map remains the layer/namespace authority."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
from uuid import uuid4

import yaml


def _inside(root, value):
    root=Path(root).absolute()
    path=Path(value)
    if not path.is_absolute():path=root/path
    # Keep the lexical ASCII workspace alias used by Humble's YAML parser.
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError('地图路径必须位于项目目录内。')
    if not path.is_relative_to(root):
        path=root/path.resolve().relative_to(root.resolve())
    return path


def _entries(root, registry_file=None):
    if registry_file is None:
        registry_file=Path(root)/'src/semantic_search/config/map_resources.json'
        if not registry_file.is_file():
            registry_file=Path(__file__).resolve().parents[1]/'config/map_resources.json'
        if not registry_file.is_file():
            from ament_index_python.packages import get_package_share_directory
            registry_file=Path(get_package_share_directory('semantic_search'))/'config/map_resources.json'
    registry=json.loads(Path(registry_file).read_text())
    if registry.get('version')!=1 or not isinstance(registry.get('maps'),dict):
        raise ValueError('地图资源注册表版本或结构无效。')
    entries={};owned=set()
    for key,entry in registry['maps'].items():
        if not isinstance(entry,dict) or not isinstance(entry.get('yaml'),str):
            raise ValueError('地图资源缺少主YAML。')
        aliases=entry.get('compatible_yaml',[])
        if not isinstance(aliases,list) or not all(isinstance(p,str) and p for p in aliases):
            raise ValueError('地图兼容路径列表无效。')
        main=_inside(root,entry['yaml']);paths=[main]+[_inside(root,p) for p in aliases]
        resolved={p.resolve() for p in paths}
        if resolved & owned:
            raise ValueError('不同地图版本不能共用注册路径。')
        owned.update(resolved);entries[key]=(main,paths)
    return entries


def _selection(root, map_id=None, yaml_path=None, registry_file=None):
    if (map_id is None)==(yaml_path is None):
        raise ValueError('请指定一个地图资源ID或YAML路径。')
    entries=_entries(root,registry_file)
    if map_id is not None:
        if map_id not in entries:raise ValueError('未知地图资源ID：'+str(map_id))
        main,paths=entries[map_id]
        return map_id,main,main,paths
    requested=_inside(root,yaml_path)
    for key,(main,paths) in entries.items():
        if requested.resolve() in {p.resolve() for p in paths}:
            return key,main,requested,paths
    # Explicit new versions stay separate; never match them by pixels alone.
    return None,requested,requested,[requested]


def canonical_map_yaml(root, *, map_id=None, yaml_path=None, registry_file=None):
    """Resolve selection only, so a scene can still build a missing new map."""
    return _selection(root,map_id,yaml_path,registry_file)[1]


def _inspect(root, path):
    path=_inside(root,path);contents=path.read_bytes();metadata=yaml.safe_load(contents)
    if not isinstance(metadata,dict) or not isinstance(metadata.get('image'),str) or not metadata['image']:
        raise ValueError('地图YAML缺少有效image路径。')
    resolution=metadata.get('resolution');origin=metadata.get('origin')
    if (type(resolution) not in (int,float) or not math.isfinite(resolution) or resolution<=0
            or not isinstance(origin,list) or len(origin)!=3
            or not all(type(v) in (int,float) and math.isfinite(v) for v in origin)):
        raise ValueError('地图分辨率与原点必须有效且有限。')
    image=_inside(root,path.parent/metadata['image'])
    return dict(yaml=path,image=image,metadata=metadata,
                yaml_sha256=hashlib.sha256(contents).hexdigest(),
                image_sha256=hashlib.sha256(image.read_bytes()).hexdigest())


def _geometry(resource):
    return {k:v for k,v in resource['metadata'].items() if k!='image'}


def resolve_map(root, *, map_id=None, yaml_path=None, registry_file=None):
    """Verify existing aliases before redirecting them to the canonical map."""
    key,main,requested,paths=_selection(root,map_id,yaml_path,registry_file)
    resource=_inspect(root,main);verified=[]
    for path in paths:
        if path==main:other=resource
        elif not path.exists():continue
        else:other=_inspect(root,path)
        if _geometry(other)!=_geometry(resource) or other['image_sha256']!=resource['image_sha256']:
            raise ValueError('同版地图兼容副本已分歧，保留原件并拒绝混用：'+str(path))
        verified.append(str(path))
    original=resource if requested==main else _inspect(root,requested)
    resource.update(map_id=key,requested_yaml=requested,
                    requested_yaml_sha256=original['yaml_sha256'],verified_copies=verified)
    return resource


def map_receipt(resource, root):
    root=Path(root).absolute()
    receipt={k:resource[k] for k in ['map_id','metadata','yaml_sha256','image_sha256','requested_yaml_sha256']}
    for key in ['yaml','image','requested_yaml']:
        receipt[key]=str(resource[key].relative_to(root))
    receipt['verified_copies']=[str(Path(p).relative_to(root)) for p in resource['verified_copies']]
    return receipt


def same_map(first,second):
    return _geometry(first)==_geometry(second) and first['image_sha256']==second['image_sha256']


def new_map_prefix(root, scene, prefix=None, registry_file=None):
    """Choose a new output version; never overwrite a registered or saved map."""
    if not isinstance(scene,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}',scene):
        raise ValueError('地图场景标识无效。')
    if prefix is None:
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_'+uuid4().hex[:8]
        prefix=Path('results/maps')/scene/stamp/'map'
    path=_inside(root,prefix)
    path=Path(root).absolute()/path.resolve().relative_to(Path(root).resolve())
    if path.suffix in ('.yaml','.yml','.pgm','.png'):
        raise ValueError('地图输出应为文件前缀，不能带地图扩展名。')
    protected={p.resolve().with_suffix('') for _,paths in _entries(root,registry_file).values() for p in paths}
    outputs=[_inside(root,str(path)+suffix) for suffix in ['.yaml','.yml','.pgm','.png']]
    if path.resolve() in protected or path.exists() or any(p.exists() for p in outputs):
        raise ValueError('地图输出已存在或属于保留地图，请选择新版本路径。')
    return path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    choice=parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--map-id');choice.add_argument('--yaml-path',type=Path)
    choice.add_argument('--new-output',action='store_true')
    parser.add_argument('--scene',default='aws_house');parser.add_argument('--output-prefix',type=Path)
    parser.add_argument('--json',action='store_true')
    args=parser.parse_args()
    if args.new_output:
        print(new_map_prefix(args.root,args.scene,args.output_prefix));return
    resource=resolve_map(args.root,map_id=args.map_id,yaml_path=args.yaml_path)
    print(json.dumps(map_receipt(resource,args.root),ensure_ascii=False) if args.json else resource['yaml'])


if __name__=='__main__':main()
