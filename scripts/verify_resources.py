#!/usr/bin/env python3
"""Validate prepared files and package metadata without starting task experiments."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-baseline', action='store_true')
    parser.add_argument('--output', default='resources/preparation_status.json')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    read = lambda path: json.loads((root/path).read_text())
    checks = {}
    assets = read('resources/assets.lock.json')['assets']
    for asset in assets:
        folder = root/asset['directory']
        checks['asset_'+asset['id']] = (
            digest(root/asset['archive']) == asset['sha256'] and
            digest(folder/'model.sdf') == asset['model_sdf_sha256'])
        ET.parse(folder/'model.sdf')
    model = read('resources/voice_model.lock.json')
    checks['voice_model_files'] = all(
        digest(root/model['directory']/item['path']) == item['sha256'] for item in model['files'])
    audio = read('resources/native_audio.lock.json')
    checks['native_audio_archive'] = digest(root/audio['archive']) == audio['sha256']
    checks['native_audio_library'] = (root/'resources/native/usr/lib/x86_64-linux-gnu/libportaudio.so.2').is_file()
    camera = read('resources/camera_reference.lock.json')
    reference = root/camera['directory']
    head = subprocess.check_output(['git','-C',str(reference),'rev-parse','HEAD'],text=True).strip()
    checks['camera_reference'] = (head == camera['commit'] and (reference/'LICENSE').is_file()
                                  and (reference/'realsense2_description/urdf/_d435.urdf.xacro').is_file())
    output = subprocess.check_output([
        str(root/'.venv-voice/bin/python'),'-m','pip','--isolated','--no-cache-dir','--disable-pip-version-check',
        'list','--local','--format=json'],text=True)
    normalize = lambda value: re.sub('[-_.]+','-',value).lower()
    installed = {normalize(item['name']):item['version'] for item in json.loads(output)}
    wanted = dict(line.split('==',1) for line in (root/'requirements-voice-ubuntu-cpu.txt').read_text().splitlines()
                  if line and not line.startswith('#'))
    checks['voice_packages'] = all(installed.get(normalize(name)) == version for name,version in wanted.items())
    wheels = read('resources/voice_wheels.lock.json')['wheels']
    wheel_versions = {normalize(item['name']):item['version'] for item in wheels}
    checks['offline_voice_wheels'] = (
        all(digest(root/item['path']) == item['sha256'] for item in wheels) and
        all(wheel_versions.get(normalize(name)) == version for name,version in wanted.items()))
    packages = ['ros-humble-nav2-smac-planner','ros-humble-nav2-mppi-controller',
                'ros-humble-nav2-regulated-pure-pursuit-controller']
    rows = subprocess.check_output(['dpkg-query','-W','-f=${Package}\t${Status}\t${Version}\n',*packages],text=True)
    versions = {parts[0]:parts[2] for row in rows.splitlines()
                if len(parts := row.split('\t')) == 3 and parts[1] == 'install ok installed'}
    checks['navigation_plugins_installed'] = all(package in versions for package in packages)
    if args.check_baseline:
        expected = read('resources/preparation_baseline.sha256.json')
        checks['active_baseline_unchanged'] = all(digest(root/path) == sha for path,sha in expected.items())
    report = dict(status='prepared' if all(checks.values()) else 'incomplete', checks=checks,
                  verified_at_utc=datetime.now(timezone.utc).isoformat(),
                  target_assets=len(assets), navigation_packages=versions,
                  voice_environment='.venv-voice', voice_model_revision=model['revision'],
                  functional_experiments_run=False, speech_inference_tested=False,
                  audio_recording_tested=False, assets_inserted_into_scene=False,
                  camera_reference_activated=False)
    destination = root/args.output
    destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    raise SystemExit(0 if all(checks.values()) else 1)


if __name__ == '__main__':
    main()
