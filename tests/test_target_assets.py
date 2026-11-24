"""Selected YCB restoration preserves provenance and rejects unsafe archives."""
import hashlib
import io
import json
from pathlib import Path
import tarfile

import pytest

import prepare_target_assets as assets


@pytest.fixture
def library(tmp_path,monkeypatch):
    monkeypatch.setattr(assets,'ROOT',tmp_path)
    archive=tmp_path/'downloads/archives';archive.mkdir(parents=True)
    models=tmp_path/'third_party/target_assets';models.mkdir(parents=True)
    item=dict(id='ycb_apple',category='apple',owner='YCB',name='013_apple',source='ycb',
              archive_url='https://ycb-benchmarks.s3.amazonaws.com/data/google/013_apple_google_16k.tgz')
    return archive,models,item


def bundle(path, members):
    with tarfile.open(path,'w:gz') as archive:
        for name,value in members:
            info=tarfile.TarInfo(name)
            if value is None:
                info.type=tarfile.SYMTYPE;info.linkname='/etc/passwd';archive.addfile(info)
            else:
                info.size=len(value);archive.addfile(info,io.BytesIO(value))


def test_locked_restore_is_offline_and_keeps_original_mesh_and_texture(library,monkeypatch):
    archives,models,item=library
    archive=archives/'ycb_apple_google_16k.tgz'
    mesh=b'v 0 0 0\nv 0.1 0 0\nv 0 0.1 0\nf 1 2 3\n'
    bundle(archive,[('013_apple/google_16k/textured.obj',mesh),
                    ('013_apple/google_16k/texture_map.png',b'original-texture')])
    monkeypatch.setattr(assets,'fetch',lambda _:pytest.fail('No network expected for cached asset'))
    first=assets.prepare_ycb(item,None,archives,models)
    assert (models/'ycb_apple/google_16k/textured.obj').read_bytes()==mesh
    source=json.loads((models/'ycb_apple/SOURCE.json').read_text())
    assert source['sha256']==hashlib.sha256(archive.read_bytes()).hexdigest()
    assert 'CC BY 4.0' in (models/'ycb_apple/ATTRIBUTION.md').read_text()
    assert assets.prepare_ycb(item,first,archives,models)['model_sdf_sha256']==first['model_sdf_sha256']
    archive.write_bytes(b'changed')
    with pytest.raises(RuntimeError,match='Archive checksum'):
        assets.prepare_ycb(item,first,archives,models)


@pytest.mark.parametrize('name,value',[('../escaped',b'data'),('/absolute/escaped',b'data'),
                                     ('013_apple/google_16k/link',None)])
def test_archive_paths_and_links_cannot_escape_model_staging(library,name,value):
    archives,models,item=library
    bundle(archives/'ycb_apple_google_16k.tgz',[(name,value)])
    with pytest.raises(ValueError,match='Unsafe YCB archive'):
        assets.prepare_ycb(item,None,archives,models)
    assert not (models/'ycb_apple').exists()
