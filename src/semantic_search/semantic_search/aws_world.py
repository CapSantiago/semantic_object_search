"""Reuse pinned AWS scene instances with Fortress systems supplied by this project."""
from copy import deepcopy
from pathlib import Path
import hashlib
import re
import xml.etree.ElementTree as ET


def adapt_mesh(source, asset_root, cache_dir):
    """Repair Collada texture paths in a derived file; never edit the vendor tree."""
    source = Path(source).resolve()
    if source.suffix.lower() != '.dae':
        return source
    destination = Path(cache_dir) / (hashlib.sha256(str(source).encode()).hexdigest()[:16] + '.dae')
    if destination.exists():
        return destination
    def texture(match):
        name = match.group(1).strip()
        path = (source.parent / name).resolve()
        if not path.is_file():
            path = Path(asset_root).resolve() / 'photos' / Path(name).name
        if not path.is_file():
            raise FileNotFoundError(f'{source}: missing texture {name}')
        return '<init_from>' + str(path) + '</init_from>'
    data = re.sub(r'<init_from>(.*?)</init_from>', texture, source.read_text(), flags=re.S)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(data)
    return destination


def import_aws_house(world, asset_root, cache_dir=None):
    asset_root = Path(asset_root)
    source = ET.parse(asset_root / 'worlds/small_house.world').getroot().find('world')
    truth = []
    for instance in source.findall('model'):
        uri = instance.findtext('include/uri')
        if not uri or not uri.startswith('model://'):
            raise ValueError(f'Unsupported AWS instance: {instance.get("name")}')
        model_file = asset_root / 'models' / uri.removeprefix('model://') / 'model.sdf'
        model = ET.parse(model_file).getroot().find('model')
        if model is None:
            raise ValueError(f'Missing model: {model_file}')
        model.set('name', instance.get('name'))
        for tag in ('static', 'pose'):
            for child in model.findall(tag):
                model.remove(child)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = instance.findtext('pose', '0 0 0 0 0 0')
        # These are static fixtures. Remove incomplete Classic inertias rather
        # than inventing dynamics; upstream originals remain untouched.
        for link in model.findall('.//link'):
            for inertial in link.findall('inertial'):
                link.remove(inertial)
        for visual in model.findall('.//visual'):
            # Classic's GUI layer tags must not affect Fortress camera visibility.
            for meta in visual.findall('meta'):
                visual.remove(meta)
        if uri.endswith('FloorB_01'):
            # A zero-thickness mesh is not a reliable DART contact surface.
            # The Fortress ground plane supplies the floor collision at z=0.
            for link in model.findall('link'):
                for collision in link.findall('collision'):
                    link.remove(collision)
        for resource in model.findall('.//uri'):
            if resource.text and resource.text.startswith('model://'):
                resolved = asset_root / 'models' / resource.text.removeprefix('model://')
                if not resolved.exists():
                    raise FileNotFoundError(resolved)
                if cache_dir is not None:
                    resolved = adapt_mesh(resolved, asset_root, cache_dir)
                resource.text = str(resolved.resolve())
        world.append(model)
        truth.append(dict(name=model.get('name'), asset=uri.removeprefix('model://'),
                          pose=[float(x) for x in model.findtext('pose').split()]))
    for tag in ('scene',):
        existing = world.find(tag)
        if existing is not None:
            world.remove(existing)
        if source.find(tag) is not None:
            world.append(deepcopy(source.find(tag)))
    for light in source.findall('light'):
        # Replace matching lights rather than creating duplicate names.
        for existing in world.findall('light'):
            if existing.get('name') == light.get('name'):
                world.remove(existing)
        copied = deepcopy(light)
        copied.find('cast_shadows').text = 'false'
        copied.find('pose').attrib.pop('frame', None)
        world.append(copied)
    return truth
