#!/usr/bin/env python3
"""Download/check fixed CTranslate2 data files. Never instantiate a speech model."""
import argparse
import hashlib
import json
import os
from pathlib import Path


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offline', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    lock = root/'resources/voice_model.lock.json'
    record = json.loads(lock.read_text())
    destination = root/record['directory']
    complete = all(item.get('sha256') and (destination/item['path']).is_file() and
                   digest(destination/item['path']) == item['sha256']
                   for item in record['files'])
    if complete and all((destination/item['path']).is_file() for item in record['files']):
        print('Offline speech files already present and checksummed; no inference performed')
        return
    if args.offline:
        raise RuntimeError('Speech files missing or changed; offline preparation cannot fetch them')
    os.environ['HF_HUB_DISABLE_IMPLICIT_TOKEN'] = '1'
    os.environ['HF_HUB_DISABLE_XET'] = '1'
    from huggingface_hub import snapshot_download
    snapshot_download(repo_id=record['repo_id'], revision=record['revision'],
                      local_dir=str(destination), cache_dir=str(root/'downloads/hf_cache'),
                      allow_patterns=[item['path'] for item in record['files']],
                      max_workers=2, token=False)
    for item in record['files']:
        if digest(destination/item['path']) != item['sha256']:
            raise RuntimeError('Speech file checksum mismatch: '+item['path'])
    print('Speech files downloaded and checksummed; no inference performed')


if __name__ == '__main__':
    main()
