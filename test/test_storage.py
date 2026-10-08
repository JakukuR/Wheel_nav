from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3

import pytest
import yaml

from wla_r680_navigation_py.storage import (
    accept_navigation_map_edit, load_storage_config, publish_navigation_map,
    resolve_navigation_map, _pgm_geometry)


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
        'semantic.geojson', 'navigation.yaml', 'manifest.json'}
    startup = yaml.safe_load((second/'navigation.yaml').read_text())
    assert startup['localization_database'] == 'rtabmap.db'
    assert startup['default_initial_pose'] is None
    semantic = json.loads((second/'semantic.geojson').read_text())
    assert semantic['features'] == []
    resolved = resolve_navigation_map(settings)
    assert resolved['map_id'] == second.name
    assert Path(resolved['database_path']) == second/'rtabmap.db'
    assert Path(resolved['map_yaml']) == second/'map.yaml'
    assert Path(resolved['startup_manifest']) == second/'navigation.yaml'


def test_checksum_rejects_modified_navigation_file(tmp_path):
    archive = tmp_path/'archive'
    make_archive(archive)
    settings = storage(tmp_path)
    published = publish_navigation_map(
        archive, settings, 'run-a', now=datetime(2026, 9, 20, tzinfo=timezone.utc))
    (published/'map.pgm').write_bytes(b'changed')
    with pytest.raises(ValueError, match='checksum mismatch'):
        resolve_navigation_map(settings, published.name)


def test_accept_intentional_pgm_edit_keeps_geometry_and_backups(tmp_path):
    archive = tmp_path/'archive'
    make_archive(archive)
    settings = storage(tmp_path)
    published = publish_navigation_map(archive, settings, 'run-a')
    before = (published/'manifest.json').read_bytes()
    edited = b'P5\n# Created by GIMP\n2 2\n255\n\xfe\xfe\xcd\xfe'
    (published/'map.pgm').write_bytes(edited)
    result = accept_navigation_map_edit(settings, published.name, 'Remove confirmed noise')
    assert result['changed']
    backup = Path(result['backup'])
    assert (backup/'manifest.before.json').read_bytes() == before
    assert (backup/'map.accepted.pgm').read_bytes() == edited
    assert (archive/'map.pgm').read_bytes() != edited
    assert resolve_navigation_map(settings, published.name)['map_id'] == published.name
    assert not accept_navigation_map_edit(settings, published.name, 'No new edits')['changed']


@pytest.mark.parametrize('damage', ['dimensions', 'truncated', 'db', 'yaml', 'metadata'])
def test_edit_acceptance_rejects_invalid_bundle_without_writing(tmp_path, damage):
    archive = tmp_path/'archive'
    make_archive(archive)
    settings = storage(tmp_path)
    published = publish_navigation_map(archive, settings, 'run-a')
    (published/'map.pgm').write_bytes(b'P5\n2 2\n255\n\xfe\xfe\xcd\xfe')
    if damage == 'dimensions':
        (published/'map.pgm').write_bytes(b'P5\n1 4\n255\n\xfe\xfe\xcd\xfe')
    elif damage == 'truncated':
        (published/'map.pgm').write_bytes(b'P5\n2 2\n255\n\xfe')
    else:
        name = {'db': 'rtabmap.db', 'yaml': 'map.yaml', 'metadata': 'map_info.json'}[damage]
        with (published/name).open('ab') as stream:
            stream.write(b' ')
    before = (published/'manifest.json').read_bytes()
    with pytest.raises(ValueError):
        accept_navigation_map_edit(settings, published.name, 'Test')
    assert (published/'manifest.json').read_bytes() == before
    assert not list(published.glob('map-edit-*'))


@pytest.mark.parametrize('selection,reason', [('latest', 'Test'), (None, 'Test'), ('map-2026-10-08-1', ' ')])
def test_edit_acceptance_requires_explicit_selection_and_reason(tmp_path, selection, reason):
    with pytest.raises(ValueError):
        accept_navigation_map_edit(storage(tmp_path), selection, reason)


def test_pgm_parser_does_not_strip_whitespace_valued_pixels():
    assert _pgm_geometry(b'P5\n2 2\n255\n\n\t\r ') == (2, 2)
    assert _pgm_geometry(b'P5\r\n2 2\r\n255\r\n\n\t\r ') == (2, 2)


def test_publish_preserves_mapping_semantics_and_home(tmp_path):
    run_dir = tmp_path/'run'
    archive = run_dir/'map_archive'
    run_dir.mkdir()
    make_archive(archive)
    candidate = dict(type='FeatureCollection', schema='wla-semantic-map-v1',
                     map_id='run-a', revision=2, features=[dict(
                         type='Feature', id='home',
                         geometry=dict(type='Point', coordinates=[0.0, 0.0, 0.0]),
                         properties=dict(role='home', label='出生点'))])
    (run_dir/'semantic.geojson').write_text(json.dumps(candidate))
    published = publish_navigation_map(
        archive, storage(tmp_path), 'run-a',
        now=datetime(2026, 9, 20, tzinfo=timezone.utc))
    copied = json.loads((published/'semantic.geojson').read_text())
    assert copied['map_id'] == published.name
    assert copied['features'][0]['properties']['role'] == 'home'
    assert copied['features'][0]['geometry']['coordinates'] == [0.0, 0.0, 0.0]


def test_publish_rejects_semantics_from_another_run(tmp_path):
    run_dir = tmp_path/'run'
    archive = run_dir/'map_archive'
    run_dir.mkdir()
    make_archive(archive)
    (run_dir/'semantic.geojson').write_text(json.dumps(dict(
        type='FeatureCollection', schema='wla-semantic-map-v1',
        map_id='run-elsewhere', revision=0, features=[])))
    with pytest.raises(ValueError, match='identity mismatch'):
        publish_navigation_map(archive, storage(tmp_path), 'run-a',
                               now=datetime(2026, 9, 20, tzinfo=timezone.utc))
