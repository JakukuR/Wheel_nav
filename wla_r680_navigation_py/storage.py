"""Versioned full-run storage and compact navigation-map publishing."""
from datetime import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tempfile

import yaml

from .archive import validate_database


def _resolve_path(value, base):
    path = Path(value).expanduser()
    return (path if path.is_absolute() else Path(base) / path).resolve()


def load_storage_config(path):
    path = Path(path).expanduser().resolve()
    value = yaml.safe_load(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f'invalid storage config: {path}')
    for key in ('run_root', 'maps_root', 'map_prefix', 'active_map'):
        if not isinstance(value.get(key), str) or not value[key].strip():
            raise ValueError(f'invalid storage setting: {key}')
    prefix = value['map_prefix'].strip()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', prefix):
        raise ValueError('map_prefix must contain only letters, digits, _ or -')
    active = value['active_map'].strip()
    if active != 'latest' and not re.fullmatch(
            rf'{re.escape(prefix)}-\d{{4}}-\d{{2}}-\d{{2}}-[1-9]\d*', active):
        raise ValueError('active_map must be latest or a generated map name')
    return dict(run_root=_resolve_path(value['run_root'], path.parent),
                maps_root=_resolve_path(value['maps_root'], path.parent),
                map_prefix=prefix, active_map=active, config_path=path)


def _fsync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _validate_map_name(name, prefix):
    if not re.fullmatch(rf'{re.escape(prefix)}-\d{{4}}-\d{{2}}-\d{{2}}-[1-9]\d*', name):
        raise ValueError(f'invalid navigation map name: {name}')


def _next_map_name(root, prefix, date):
    pattern = re.compile(rf'{re.escape(prefix)}-{re.escape(date)}-([1-9]\d*)')
    used = []
    for path in root.iterdir():
        match = pattern.fullmatch(path.name)
        if path.is_dir() and match:
            used.append(int(match.group(1)))
    return f'{prefix}-{date}-{max(used, default=0)+1}'


def publish_navigation_map(archive, storage, source_run_id, map_name=None, now=None):
    """Atomically publish navigation-only files from a completed full archive."""
    archive = Path(archive).expanduser().resolve()
    root = Path(storage['maps_root']).expanduser().resolve()
    prefix = storage['map_prefix']
    required = ('map.pgm', 'map.yaml', 'occupancy_grid.json', 'rtabmap.db')
    for name in required:
        if not (archive/name).is_file():
            raise FileNotFoundError(archive/name)
    validate_database(archive/'rtabmap.db')
    grid = json.loads((archive/'occupancy_grid.json').read_text())
    if grid.get('frame_id') != 'map' or not all(key in grid for key in
                                                ('width', 'height', 'resolution', 'origin')):
        raise ValueError('invalid archived occupancy grid metadata')

    root.mkdir(parents=True, exist_ok=True)
    lock_path = root/'.publish.lock'
    with lock_path.open('a+') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        stamp = now or datetime.now().astimezone()
        name = map_name or _next_map_name(root, prefix, stamp.date().isoformat())
        _validate_map_name(name, prefix)
        destination = root/name
        if destination.exists():
            raise FileExistsError(destination)
        temporary = Path(tempfile.mkdtemp(prefix=f'.{name}.partial-', dir=root))
        try:
            for filename in ('map.pgm', 'map.yaml', 'rtabmap.db'):
                shutil.copy2(archive/filename, temporary/filename)
            validate_database(temporary/'rtabmap.db')
            info = dict(schema='wla-navigation-map-v1', map_id=name,
                        created_at=stamp.isoformat(), source_run_id=source_run_id,
                        frame_id='map', resolution=grid['resolution'],
                        width=grid['width'], height=grid['height'], origin=grid['origin'],
                        occupancy_map='map.yaml', localization_database='rtabmap.db',
                        semantic_layer='semantic.geojson')
            (temporary/'map_info.json').write_text(
                json.dumps(info, indent=2, allow_nan=False)+'\n')
            semantic = dict(type='FeatureCollection', schema='wla-semantic-map-v1',
                            map_id=name, revision=0, features=[])
            (temporary/'semantic.geojson').write_text(
                json.dumps(semantic, indent=2, allow_nan=False)+'\n')
            startup = dict(schema='wla-navigation-startup-v1', map_id=name,
                           frame_id='map', occupancy_map='map.yaml',
                           localization_database='rtabmap.db',
                           map_metadata='map_info.json',
                           semantic_layer='semantic.geojson',
                           default_initial_pose=None)
            (temporary/'navigation.yaml').write_text(yaml.safe_dump(
                startup, sort_keys=False, allow_unicode=True))
            immutable = ('map.pgm', 'map.yaml', 'rtabmap.db', 'map_info.json',
                         'navigation.yaml')
            manifest = dict(schema='wla-navigation-map-manifest-v1', map_id=name,
                            frame_id='map', files={item:_sha256(temporary/item)
                                                   for item in immutable})
            (temporary/'manifest.json').write_text(
                json.dumps(manifest, indent=2, allow_nan=False)+'\n')
            for path in temporary.iterdir():
                if path.is_file():
                    with path.open('rb') as stream:
                        os.fsync(stream.fileno())
            _fsync_directory(temporary)
            os.rename(temporary, destination)
            _fsync_directory(root)
            return destination
        except Exception:
            if temporary.exists():
                shutil.rmtree(temporary)
            raise


def resolve_navigation_map(storage, selection=None):
    """Resolve and verify the selected navigation map without modifying it."""
    root = Path(storage['maps_root']).expanduser().resolve()
    prefix = storage['map_prefix']
    selected = selection or storage['active_map']
    if not root.is_dir():
        raise FileNotFoundError(root)
    if selected == 'latest':
        pattern = re.compile(rf'{re.escape(prefix)}-(\d{{4}}-\d{{2}}-\d{{2}})-([1-9]\d*)')
        candidates = []
        for path in root.iterdir():
            match = pattern.fullmatch(path.name)
            if path.is_dir() and match:
                candidates.append((match.group(1), int(match.group(2)), path))
        if not candidates:
            raise FileNotFoundError(f'no navigation maps under {root}')
        directory = max(candidates, key=lambda item:(item[0], item[1]))[2]
    else:
        _validate_map_name(selected, prefix)
        directory = root/selected
    if not directory.is_dir():
        raise FileNotFoundError(directory)

    info = json.loads((directory/'map_info.json').read_text())
    manifest = json.loads((directory/'manifest.json').read_text())
    startup = yaml.safe_load((directory/'navigation.yaml').read_text())
    if (info.get('map_id') != directory.name or
            manifest.get('map_id') != directory.name or
            not isinstance(startup, dict) or
            startup.get('map_id') != directory.name):
        raise ValueError('navigation map identity mismatch')
    files = manifest.get('files')
    required = {'map.pgm', 'map.yaml', 'rtabmap.db', 'map_info.json',
                'navigation.yaml'}
    if not isinstance(files, dict) or set(files) != required:
        raise ValueError('navigation map manifest is incomplete')
    for name, expected in files.items():
        path = directory/name
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f'navigation map checksum mismatch: {name}')
    if startup.get('schema') != 'wla-navigation-startup-v1':
        raise ValueError('unsupported navigation startup schema')
    names = {}
    for key in ('occupancy_map', 'localization_database', 'map_metadata',
                'semantic_layer'):
        value = startup.get(key)
        if not isinstance(value, str) or Path(value).name != value:
            raise ValueError(f'invalid navigation startup path: {key}')
        names[key] = value
        if not (directory/value).is_file():
            raise FileNotFoundError(directory/value)
    validate_database(directory/names['localization_database'])
    return dict(map_id=directory.name, directory=str(directory),
                startup_manifest=str(directory/'navigation.yaml'),
                map_yaml=str(directory/names['occupancy_map']),
                database_path=str(directory/names['localization_database']),
                map_info=str(directory/names['map_metadata']),
                semantic_layer=str(directory/names['semantic_layer']),
                default_initial_pose=startup.get('default_initial_pose'))
