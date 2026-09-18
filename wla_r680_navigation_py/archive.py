"""Atomic RTAB-Map backup + standard ROS map archive; no live SQLite copy."""
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sqlite3
import tempfile
import yaml


def validate_database(path):
    with sqlite3.connect(Path(path).resolve().as_uri()+'?mode=ro', uri=True) as db:
        if db.execute('PRAGMA integrity_check').fetchone()[0] != 'ok':
            raise ValueError('database integrity check failed')
        if db.execute('SELECT count(*) FROM Node').fetchone()[0] <= 0:
            raise ValueError('empty RTAB-Map graph')


def save_archive(destination, grid, closed_backup, metadata, configs):
    """closed_backup must be produced by acknowledged /rtabmap/backup, not live db."""
    destination = Path(destination)
    if destination.exists():
        raise FileExistsError(destination)
    width, height, resolution = grid['width'], grid['height'], grid['resolution']
    cells, origin = grid['data'], grid['origin']
    if (not isinstance(width, int) or not isinstance(height, int) or min(width, height) <= 0
            or len(cells) != width*height or not math.isfinite(resolution) or resolution <= 0
            or len(origin) != 3 or not all(math.isfinite(v) for v in origin)
            or any(not isinstance(v, int) or v < -1 or v > 100 for v in cells)
            or grid.get('frame_id') != 'map'):
        raise ValueError('invalid occupancy grid')
    validate_database(closed_backup)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=destination.name+'.partial-', dir=destination.parent))
    try:
        # PGM image rows start at the top, ROS OccupancyGrid rows start at the bottom.
        pixels = bytearray()
        for row in range(height-1, -1, -1):
            for value in cells[row*width:(row+1)*width]:
                pixels.append(205 if value < 0 or 25 < value < 65 else (254 if value <= 25 else 0))
        (temporary/'map.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+pixels)
        (temporary/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=resolution,
            origin=origin, negate=0, occupied_thresh=.65, free_thresh=.25, mode='trinary')))
        (temporary/'occupancy_grid.json').write_text(json.dumps(grid, allow_nan=False))
        shutil.copyfile(closed_backup, temporary/'rtabmap.db')
        validate_database(temporary/'rtabmap.db')
        (temporary/'resolved_params.yaml').write_text(yaml.safe_dump(configs))
        (temporary/'mission_result.json').write_text(json.dumps(metadata, indent=2, allow_nan=False))
        hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in temporary.iterdir()}
        (temporary/'manifest.json').write_text(json.dumps(dict(schema='wla-map-archive-v1',
            frame_id='map', simulation_only=False, files=hashes, metadata=metadata), indent=2, allow_nan=False))
        for path in temporary.iterdir():
            with path.open('rb') as stream:
                os.fsync(stream.fileno())
        for directory in (temporary,):
            fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        os.rename(temporary, destination)
        fd = os.open(destination.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
        return destination
    except Exception as exc:
        if temporary.exists():
            (temporary/'failure.json').write_text(json.dumps({'error': str(exc)}))
        raise
