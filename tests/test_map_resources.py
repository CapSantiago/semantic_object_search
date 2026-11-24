"""One logical grid across file consumers; aliases never conceal a changed map."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from semantic_search.map_resources import (canonical_map_yaml, map_receipt,
                                          new_map_prefix, resolve_map)

ROOT=Path(__file__).resolve().parents[1]


def registry(tmp_path):
    config=tmp_path/'src/semantic_search/config';config.mkdir(parents=True)
    spec={'version':1,'maps':{'test_v1':{'yaml':'canonical/map.yaml','compatible_yaml':['legacy/map.yaml']}}}
    (config/'map_resources.json').write_text(json.dumps(spec))
    metadata=dict(image='map.pgm',resolution=.05,origin=[0,0,0],negate=0,occupied_thresh=.65,free_thresh=.25)
    for name in ['canonical','legacy']:
        folder=tmp_path/name;folder.mkdir();(folder/'map.yaml').write_text(yaml.safe_dump(metadata));(folder/'map.pgm').write_bytes(b'P5\n2 2\n255\n\xff\xff\x00\xcd')
    return config/'map_resources.json'


def test_real_scene_compatibility_paths_share_the_canonical_files_without_modification():
    before={p:hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'maps/aws_house/house.yaml',ROOT/'maps/aws_house/house.pgm']}
    current=resolve_map(ROOT,map_id='aws_house_v1')
    for alias in ['maps/aws_house/house.yaml','scenes/aws_house/maps/house.yaml']:
        old=resolve_map(ROOT,yaml_path=alias)
        assert old['yaml']==current['yaml'] and old['image_sha256']==current['image_sha256']
        assert old['map_id']=='aws_house_v1' and len(old['verified_copies'])==3
    assert all(hashlib.sha256(p.read_bytes()).hexdigest()==sha for p,sha in before.items())
    baseline=resolve_map(ROOT,map_id='baseline_v1')
    assert baseline['image_sha256']!=current['image_sha256']


@pytest.mark.parametrize('change',['pixels','origin','resolution'])
def test_alias_divergence_is_rejected_without_overwriting_either_version(tmp_path,change):
    registry(tmp_path);image=tmp_path/'legacy/map.pgm';header=tmp_path/'legacy/map.yaml'
    if change=='pixels':image.write_bytes(b'different-map')
    else:
        data=yaml.safe_load(header.read_text());data[change]=[1,0,0] if change=='origin' else .1;header.write_text(yaml.safe_dump(data))
    before={p:p.read_bytes() for p in [image,header,tmp_path/'canonical/map.yaml',tmp_path/'canonical/map.pgm']}
    with pytest.raises(ValueError,match='分歧'):resolve_map(tmp_path,yaml_path='legacy/map.yaml')
    assert all(p.read_bytes()==raw for p,raw in before.items())


def test_ascii_workspace_alias_is_retained_for_absolute_real_path_inputs(tmp_path):
    real=tmp_path/'中文项目';real.mkdir();registry(real);alias=tmp_path/'ascii';alias.symlink_to(real,target_is_directory=True)
    resource=resolve_map(alias,yaml_path=real/'legacy/map.yaml')
    assert str(resource['yaml']).isascii() and resource['yaml']==alias/'canonical/map.yaml'
    assert map_receipt(resource,alias)['yaml']=='canonical/map.yaml'


def test_new_unregistered_version_is_not_redirected_by_identical_pixels(tmp_path):
    registry(tmp_path);folder=tmp_path/'new_version';folder.mkdir()
    for ext in ['yaml','pgm']:(folder/('map.'+ext)).write_bytes((tmp_path/'canonical'/('map.'+ext)).read_bytes())
    resource=resolve_map(tmp_path,yaml_path='new_version/map.yaml')
    assert resource['map_id'] is None and resource['yaml']==folder/'map.yaml'


def test_missing_alias_is_optional_but_existing_alias_cannot_hide_a_missing_image(tmp_path):
    registry(tmp_path);(tmp_path/'legacy/map.pgm').unlink()
    with pytest.raises(FileNotFoundError):resolve_map(tmp_path,map_id='test_v1')
    (tmp_path/'legacy/map.yaml').unlink()
    resource=resolve_map(tmp_path,map_id='test_v1');assert len(resource['verified_copies'])==1
    (tmp_path/'canonical/map.yaml').unlink()
    assert canonical_map_yaml(tmp_path,map_id='test_v1')==tmp_path/'canonical/map.yaml'
    with pytest.raises(FileNotFoundError):resolve_map(tmp_path,map_id='test_v1')


def test_registered_images_cannot_escape_project_through_a_symlink(tmp_path):
    root=tmp_path/'project';root.mkdir();registry(root);outside=tmp_path/'outside.pgm';outside.write_bytes(b'outside')
    image=root/'canonical/map.pgm';image.unlink();image.symlink_to(outside)
    with pytest.raises(ValueError,match='项目目录'):resolve_map(root,map_id='test_v1')


def test_comment_only_alias_changes_preserve_geometry_but_report_exact_header_hash(tmp_path):
    registry(tmp_path);path=tmp_path/'legacy/map.yaml';path.write_text('# retained copy\n'+path.read_text())
    resource=resolve_map(tmp_path,yaml_path='legacy/map.yaml')
    assert resource['requested_yaml_sha256']!=resource['yaml_sha256']
    assert resource['requested_yaml_sha256']==hashlib.sha256(path.read_bytes()).hexdigest()


def test_saving_chooses_new_versions_and_rejects_canonical_alias_or_existing_outputs(tmp_path):
    registry(tmp_path);first=new_map_prefix(tmp_path,'aws_house');second=new_map_prefix(tmp_path,'aws_house')
    assert first!=second and first.is_relative_to(tmp_path/'results/maps/aws_house') and not first.parent.exists()
    for name in ['canonical/map','legacy/map']:
        with pytest.raises(ValueError,match='保留地图'):new_map_prefix(tmp_path,'aws_house',name)
    output=tmp_path/'outputs/map';output.parent.mkdir();output.with_suffix('.pgm').write_bytes(b'evidence')
    with pytest.raises(ValueError,match='已存在'):new_map_prefix(tmp_path,'aws_house',output)
    with pytest.raises(ValueError):new_map_prefix(tmp_path,'../escape')
    with pytest.raises(ValueError,match='项目目录'):new_map_prefix(tmp_path,'aws_house','.')


def test_cli_preserves_ascii_paths_and_validates_before_emitting(tmp_path):
    registry(tmp_path);script=ROOT/'src/semantic_search/semantic_search/map_resources.py'
    done=subprocess.run([sys.executable,str(script),'--root',str(tmp_path),'--map-id','test_v1'],capture_output=True,text=True)
    assert done.returncode==0 and done.stdout.strip()==str(tmp_path/'canonical/map.yaml')
    (tmp_path/'legacy/map.pgm').write_bytes(b'drift')
    failed=subprocess.run([sys.executable,str(script),'--root',str(tmp_path),'--map-id','test_v1'],capture_output=True,text=True)
    assert failed.returncode!=0 and not failed.stdout


def shell_fixture(tmp_path, script):
    import os
    import shutil
    import shlex
    registry(tmp_path)
    scripts=tmp_path/'scripts';scripts.mkdir()
    shutil.copyfile(ROOT/'scripts'/script,scripts/script)
    module=tmp_path/'src/semantic_search/semantic_search';module.mkdir()
    shutil.copyfile(ROOT/'src/semantic_search/semantic_search/map_resources.py',module/'map_resources.py')
    binary=tmp_path/'bin';binary.mkdir();capture=tmp_path/'argv.json'
    fake=binary/'ros2';fake.write_text('#!/usr/bin/python3\nimport json,os,sys\nopen(os.environ["MAP_ARGV_CAPTURE"],"w").write(json.dumps(sys.argv[1:]))\n');fake.chmod(0o755)
    (scripts/'env.sh').write_text('export SEMANTIC_PROJECT_ROOT='+shlex.quote(str(tmp_path))+'\nexport SEMANTIC_WS='+shlex.quote(str(tmp_path))+'\nexport PATH='+shlex.quote(str(binary))+':"$PATH"\n')
    (scripts/'native_gui_env.sh').write_text('true\n')
    (scripts/'scene_env.sh').write_text('SEMANTIC_SCENE=aws_house\nSEMANTIC_MAP="$SEMANTIC_WS/legacy/map.yaml"\nSEMANTIC_LAUNCH_ARGS=("$@")\n')
    (scripts/'start_navigation.py').write_text('pass\n')
    return scripts/script,dict(os.environ,MAP_ARGV_CAPTURE=str(capture)),capture


@pytest.mark.parametrize('name',['navigation.sh','development_navigation.sh'])
def test_navigation_shells_send_only_the_verified_canonical_yaml_to_ros(tmp_path,name):
    script,env,capture=shell_fixture(tmp_path,name)
    done=subprocess.run(['bash',str(script),'map:='+str(tmp_path/'legacy/map.yaml')],env=env,capture_output=True,text=True,timeout=8)
    assert done.returncode==0,done.stderr
    argv=json.loads(capture.read_text());maps=[arg for arg in argv if arg.startswith('map:=')]
    assert maps==['map:='+str(tmp_path/'canonical/map.yaml')]


def test_save_shell_uses_a_new_version_and_stops_before_ros_for_a_protected_prefix(tmp_path):
    script,env,capture=shell_fixture(tmp_path,'save_map.sh')
    done=subprocess.run(['bash',str(script)],env=env,capture_output=True,text=True,timeout=8)
    assert done.returncode==0,done.stderr
    argv=json.loads(capture.read_text());prefix=Path(argv[argv.index('-f')+1])
    assert prefix.is_relative_to(tmp_path/'results/maps/aws_house')
    capture.unlink()
    bad=subprocess.run(['bash',str(script),'map_output:='+str(tmp_path/'canonical/map')],env=env,capture_output=True,text=True,timeout=8)
    assert bad.returncode!=0 and not capture.exists()
