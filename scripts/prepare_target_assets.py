#!/usr/bin/env python3
"""Restore Fuel/YCB assets with source metadata and checksums, without Gazebo."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import tempfile
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def fetch(url):
    request = urllib.request.Request(url, headers={'User-Agent': 'semantic-object-search-preparation'})
    with urllib.request.urlopen(request, timeout=90) as response:
        return response.read()


def prepare_ycb(item, previous, archive_dir, library):
    """Restore three selected official YCB scans with a static SDF wrapper."""
    expected_url = ('https://ycb-benchmarks.s3.amazonaws.com/data/google/'
                    + item['name'] + '_google_16k.tgz')
    if item.get('archive_url') != expected_url:
        raise ValueError('YCB archive must match the official object and resolution.')
    archive = archive_dir/(item['id']+'_google_16k.tgz')
    if not archive.exists():
        partial = archive.with_suffix('.part')
        partial.write_bytes(fetch(expected_url))
        partial.replace(archive)
    digest = sha256(archive)
    if previous and digest != previous['sha256']:
        raise RuntimeError('Archive checksum differs from lock: '+item['id'])
    destination = library/item['id']
    if not destination.exists():
        with tempfile.TemporaryDirectory(dir=library) as staging:
            staging = Path(staging)
            with tarfile.open(archive, 'r:gz') as bundle:
                for entry in bundle.getmembers():
                    resolved = (staging/entry.name).resolve()
                    if (not resolved.is_relative_to(staging.resolve())
                            or not (entry.isfile() or entry.isdir())):
                        raise ValueError('Unsafe YCB archive member: '+entry.name)
                bundle.extractall(staging)
            meshes = list(staging.glob('*/google_16k/textured.obj'))
            if len(meshes) != 1:
                raise ValueError('Expected one YCB google_16k textured mesh.')
            prepared = staging/'prepared';prepared.mkdir()
            shutil.copytree(meshes[0].parent, prepared/'google_16k')
            sdf = ET.Element('sdf', version='1.6')
            model = ET.SubElement(sdf,'model',name=item['id'])
            ET.SubElement(model,'static').text='true'
            link = ET.SubElement(model,'link',name='object')
            for tag in ('collision','visual'):
                node = ET.SubElement(link,tag,name=tag)
                geometry = ET.SubElement(node,'geometry')
                mesh = ET.SubElement(geometry,'mesh')
                ET.SubElement(mesh,'uri').text='google_16k/textured.obj'
            ET.indent(sdf)
            ET.ElementTree(sdf).write(prepared/'model.sdf',encoding='utf-8',xml_declaration=True)
            attribution = ('# YCB Object and Model Set\n\n'
                'Berk Calli, Arjun Singh, Aaron Walsman, Siddhartha Srinivasa, Pieter Abbeel, Aaron M. Dollar.\n\n'
                'Source: https://ycb-benchmarks.s3.amazonaws.com/index.html\n\n'
                'Data license: CC BY 4.0, https://creativecommons.org/licenses/by/4.0/\n\n'
                'Original Google 16k mesh, material and textures retained. '
                'This project adds a static SDF visual/collision wrapper; no perception labels or poses are embedded.\n')
            (prepared/'ATTRIBUTION.md').write_text(attribution)
            shutil.copytree(prepared,destination)
    model_digest = sha256(destination/'model.sdf')
    if previous and model_digest != previous['model_sdf_sha256']:
        raise RuntimeError('Model checksum differs from lock: '+item['id'])
    ET.parse(destination/'model.sdf')
    record = dict(item,version='google_16k',sha256=digest,archive=str(archive.relative_to(ROOT)),
        directory=str(destination.relative_to(ROOT)),model_sdf_sha256=model_digest,
        metadata=dict(source='YCB Object and Model Set',variant='Google 16k',license='CC BY 4.0',
                      url='https://ycb-benchmarks.s3.amazonaws.com/index.html'),
        status='downloaded_not_scene_validated',active_scene_modified=False)
    (destination/'SOURCE.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout',type=Path,help='restore only already locked assets required by this layout')
    parser.add_argument('--asset',action='append',help='prepare selected asset IDs; repeatable')
    args = parser.parse_args()
    requested = json.loads((ROOT/'resources/asset_requests.json').read_text())['assets']
    manifest_path = ROOT/'resources/assets.lock.json'
    old = json.loads(manifest_path.read_text()) if manifest_path.exists() else {'assets': []}
    locked = {item['id']: item for item in old['assets']}
    if args.layout:
        layout = json.loads(args.layout.read_text())
        wanted = set()
        for target in layout['targets']:
            asset_id = target['asset']
            asset = layout.get('assets',{}).get(asset_id,{})
            directory = asset.get('directory','third_party/target_assets/'+asset_id)
            if directory == 'third_party/target_assets/'+asset_id:
                if asset_id not in locked:
                    raise ValueError('Asset is missing from the lock: '+asset_id)
                wanted.add(asset_id)
            elif not (ROOT/directory/'model.sdf').is_file():
                raise FileNotFoundError(ROOT/directory/'model.sdf')
        requested = [locked[asset_id] for asset_id in sorted(wanted)]
    if args.asset:
        available = {item['id']:item for item in requested}
        unknown = set(args.asset)-set(available)
        if unknown:
            raise ValueError('Unknown requested asset IDs: '+', '.join(sorted(unknown)))
        requested = [available[key] for key in dict.fromkeys(args.asset)]
    archive_dir = ROOT/'downloads/archives'
    archive_dir.mkdir(parents=True, exist_ok=True)
    library = ROOT/'third_party/target_assets'
    library.mkdir(parents=True, exist_ok=True)
    results = []
    for item in requested:
        print('Preparing '+item['id'], flush=True)
        if item.get('source') == 'ycb':
            previous=locked.get(item['id'])
            if previous and any(previous[key] != item[key] for key in ('owner','name','category')):
                raise ValueError('Requested YCB asset differs from lock: '+item['id'])
            record=prepare_ycb(item,previous,archive_dir,library)
            results.append(record)
            combined=dict(locked)
            combined.update({value['id']:value for value in results})
            manifest_path.write_text(json.dumps({'assets':list(combined.values())},ensure_ascii=False,indent=2)+'\n')
            print('  google_16k, sha256='+record['sha256'][:16],flush=True)
            continue
        base = 'https://fuel.gazebosim.org/1.0/'+urllib.parse.quote(item['owner'], safe='')+'/models/'+urllib.parse.quote(item['name'], safe='')
        previous = locked.get(item['id'])
        if previous and any(previous[key] != item[key] for key in ('owner', 'name', 'category')):
            raise ValueError('Requested asset differs from existing lock: '+item['id'])
        metadata = previous['metadata'] if previous else json.loads(fetch(base))
        version = metadata['version']
        url = previous['archive_url'] if previous else base+'/'+str(version)+'/'+urllib.parse.quote(item['name'], safe='')+'.zip'
        archive = archive_dir/(item['id']+'_v'+str(version)+'.zip')
        if not archive.exists():
            partial = archive.with_suffix('.zip.part')
            partial.write_bytes(fetch(url))
            partial.replace(archive)
        digest = sha256(archive)
        if previous and digest != previous['sha256']:
            raise RuntimeError('Archive checksum differs from lock: '+item['id'])
        destination = library/item['id']
        if not destination.exists():
            with tempfile.TemporaryDirectory(dir=library) as staging:
                target = Path(staging)
                with zipfile.ZipFile(archive) as bundle:
                    for entry in bundle.infolist():
                        resolved = (target/entry.filename).resolve()
                        if not resolved.is_relative_to(target.resolve()):
                            raise ValueError('Unsafe archive entry: '+entry.filename)
                        if (entry.external_attr >> 16) & 0o170000 == 0o120000:
                            raise ValueError('Archive symlink not accepted: '+entry.filename)
                    bundle.extractall(target)
                models = list(target.rglob('model.sdf'))
                if len(models) != 1:
                    raise RuntimeError('Expected one model.sdf: '+item['id'])
                shutil.copytree(models[0].parent, destination)
        model = destination/'model.sdf'
        ET.parse(model)
        model_digest = sha256(model)
        if previous and model_digest != previous['model_sdf_sha256']:
            raise RuntimeError('Model checksum differs from lock: '+item['id'])
        if args.layout:
            (destination/'SOURCE.json').write_text(json.dumps(previous,ensure_ascii=False,indent=2)+'\n')
            print(f"  version={version}, sha256={digest[:16]}, {archive.stat().st_size} bytes",flush=True)
            continue
        record = dict(item, version=version, archive_url=url, sha256=digest,
                      archive=str(archive.relative_to(ROOT)), directory=str(destination.relative_to(ROOT)),
                      model_sdf_sha256=model_digest, metadata=metadata,
                      status='downloaded_not_scene_validated', active_scene_modified=False)
        (destination/'SOURCE.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
        results.append(record)
        # Persist completed entries even if a later network request fails.
        combined = {key:value for key,value in locked.items()}
        combined.update({value['id']:value for value in results})
        manifest_path.write_text(json.dumps({'assets':list(combined.values())},ensure_ascii=False,indent=2)+'\n')
        print(f"  version={version}, sha256={digest[:16]}, {archive.stat().st_size} bytes", flush=True)


if __name__ == '__main__':
    main()
