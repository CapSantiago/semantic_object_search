#!/usr/bin/env python3
"""Restore pinned runtime assets; never starts ROS, Gazebo or inference."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def download(url, destination, expected):
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if digest(destination) != expected:
            raise RuntimeError('Existing file differs from lock: ' + str(destination))
        return
    with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as stream:
        temporary = Path(stream.name)
        try:
            request = urllib.request.Request(url, headers={'User-Agent': 'semantic-object-search'})
            with urllib.request.urlopen(request, timeout=120) as response:
                shutil.copyfileobj(response, stream)
            stream.close()
            if digest(temporary) != expected:
                raise RuntimeError('Downloaded checksum differs from lock: ' + url)
            temporary.replace(destination)
        finally:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--references', action='store_true', help='also restore optional design references')
    parser.add_argument('--check', action='store_true', help='check local files only; no network or writes')
    lock = json.loads((ROOT / 'resources/runtime.lock.json').read_text())
    parser.add_argument('--model',choices=(lock['vision_model']['path'],*lock.get('candidate_vision_models',{})),default='yolov8n-seg.pt',
                        help='also prepare a locked candidate model; default keeps the n baseline')
    args = parser.parse_args()
    for record in lock['repositories']:
        if record['role'] == 'reference' and not args.references:
            continue
        folder = ROOT / record['directory']
        if not folder.exists() and not args.check:
            folder.parent.mkdir(parents=True, exist_ok=True)
            # Build in a temporary directory so a failed fetch cannot leave a partial checkout.
            with tempfile.TemporaryDirectory(dir=folder.parent) as staging:
                checkout = Path(staging) / 'checkout'
                subprocess.run(['git', 'init', str(checkout)], check=True)
                subprocess.run(['git', '-C', str(checkout), 'remote', 'add', 'origin', record['url']], check=True)
                subprocess.run(['git', '-C', str(checkout), 'fetch', '--depth', '1', 'origin', record['commit']], check=True)
                subprocess.run(['git', '-C', str(checkout), 'checkout', '--detach', 'FETCH_HEAD'], check=True)
                checkout.rename(folder)
        head = subprocess.check_output(['git', '-C', str(folder), 'rev-parse', 'HEAD'], text=True).strip()
        if head != record['commit']:
            raise RuntimeError('Existing checkout differs from lock: ' + str(folder))
        print(record['directory'] + ': ' + head)
    chair = lock['chair']
    destination = ROOT / 'third_party/fuel_chair'
    if not destination.exists() and not args.check:
        archive = ROOT / 'downloads/archives/chair_v3.zip'
        download(chair['url'], archive, chair['sha256'])
        with tempfile.TemporaryDirectory(dir=destination.parent) as staging:
            target = Path(staging)
            with zipfile.ZipFile(archive) as bundle:
                for item in bundle.infolist():
                    if not (target / item.filename).resolve().is_relative_to(target.resolve()):
                        raise ValueError('Unsafe archive path: ' + item.filename)
                    if (item.external_attr >> 16) & 0o170000 == 0o120000:
                        raise ValueError('Archive symlink: ' + item.filename)
                bundle.extractall(target)
            models = list(target.rglob('model.sdf'))
            if len(models) != 1:
                raise RuntimeError('Expected exactly one Chair model.sdf')
            shutil.copytree(models[0].parent, destination)
        (destination / 'SOURCE.json').write_text(json.dumps(chair, ensure_ascii=False, indent=2) + '\n')
        (destination / 'ATTRIBUTION.md').write_text(lock['chair_attribution'])
    if digest(destination / 'model.sdf') != chair['model_sdf_sha256']:
        raise RuntimeError('Chair model differs from lock')
    model = lock['vision_model']
    path = ROOT / model['path']
    if not args.check:
        download(model['url'], path, model['sha256'])
    if digest(path) != model['sha256']:
        raise RuntimeError('Vision weight differs from lock')
    if args.model!=model['path']:
        candidate = lock['candidate_vision_models'][args.model]
        path = ROOT/candidate['path']
        if not args.check:
            download(candidate['url'],path,candidate['sha256'])
        if digest(path)!=candidate['sha256']:
            raise RuntimeError('Candidate vision weight differs from lock')
    prompt=lock.get('vision_prompt_embeddings',{}).get(args.model)
    if prompt and (not (ROOT/prompt['path']).is_file() or digest(ROOT/prompt['path'])!=prompt['sha256']):
        raise RuntimeError('Frozen prompt embeddings differ from lock; restore the versioned resource, do not overwrite it')
    for folder, stem in [('baseline', 'course_house'), ('aws_house', 'house')]:
        for extension in ('yaml', 'pgm'):
            archived = ROOT / f'scenes/{folder}/maps/{stem}.{extension}'
            active = ROOT / f'maps/{folder}/{stem}.{extension}'
            if digest(archived) != digest(active):
                raise RuntimeError('Active map differs from archived acceptance map: ' + str(active))
    print('Pinned runtime resources checked; no functional experiment was run.')


if __name__ == '__main__':
    main()
