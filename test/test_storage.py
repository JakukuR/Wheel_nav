from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3

import pytest
import yaml

from wla_r680_navigation_py.storage import (
    load_storage_config, publish_navigation_map, resolve_navigation_map)


def make_archive(path):
    path.mkdir()
    grid = dict(frame_id='map', width=2, height=2, resolution=0.05,
                origin=[-1.0, -2.0, 0.0], data=[0, 100, -1, 0])
    (path/'occupancy_grid.json').write_text(json.dumps(grid))
    (path/'map.pgm').write_bytes(b'P5\n2 2\n255\n\xfe\x00\xcd\xfe')
    (path/'map.yaml').write_text(yaml.safe_dump(dict(
        image='map.pgm', resolution=.05, origin=grid['origin'], negate=0,
        occupied_thresh=.65, free_thresh=.25, mode='trinary')))
    with sqlite3.connect(path/'rtabmap.db') as db:
        db.execute('CREATE TABLE Node(id INTEGER PRIMARY KEY)')
        db.execute('INSERT INTO Node VALUES(1)')


def storage(tmp_path):
    return dict(run_root=tmp_path/'runs', maps_root=tmp_path/'maps',
                map_prefix='map', active_map='latest')


def test_config_paths_expand_relative_to_config(tmp_path):
    config = tmp_path/'storage.yaml'
    config.write_text('run_root: runs\nmaps_root: maps\nmap_prefix: map\nactive_map: latest\n')
    value = load_storage_config(config)
    assert value['run_root'] == (tmp_path/'runs').resolve()
    assert value['maps_root'] == (tmp_path/'maps').resolve()


def test_publish_sequence_and_resolve_latest(tmp_path):
    archive = tmp_path/'archive'
    make_archive(archive)
    settings = storage(tmp_path)
    now = datetime(2026, 9, 20, 12, 0, tzinfo=timezone.utc)
    first = publish_navigation_map(archive, settings, 'run-a', now=now)
    second = publish_navigation_map(archive, settings, 'run-b', now=now)
    assert first.name == 'map-2026-09-20-1'
    assert second.name == 'map-2026-09-20-2'
    assert {p.name for p in second.iterdir()} == {
        'map.pgm', 'map.yaml', 'rtabmap.db', 'map_info.json',
        'semantic.geojson', 'manifest.json'}
    semantic = json.loads((second/'semantic.geojson').read_text())
    assert semantic['features'] == []
    resolved = resolve_navigation_map(settings)
    assert resolved['map_id'] == second.name
    assert Path(resolved['database_path']) == second/'rtabmap.db'
    assert Path(resolved['map_yaml']) == second/'map.yaml'


def test_checksum_rejects_modified_navigation_file(tmp_path):
    archive = tmp_path/'archive'
    make_archive(archive)
    settings = storage(tmp_path)
    published = publish_navigation_map(
        archive, settings, 'run-a', now=datetime(2026, 9, 20, tzinfo=timezone.utc))
    (published/'map.pgm').write_bytes(b'changed')
    with pytest.raises(ValueError, match='checksum mismatch'):
        resolve_navigation_map(settings, published.name)
