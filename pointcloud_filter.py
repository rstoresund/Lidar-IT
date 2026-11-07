"""PointCloud Filter 3.0 -- SDK-free LAS / E57 terrain thinning, Windows GUI.

Requires numpy, laspy, pye57, customtkinter (FARO runtime is NOT required).
E57 low-level chunked reader is based on public pye57/libE57Format API;
validate on your real files before production use.
"""
from __future__ import annotations
import csv
import datetime
import json
import math
import os
import queue
import shutil
import sqlite3
import struct
import tempfile
import threading
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterator

# Run dependency check *before* importing NumPy. This also works when launched
# from Python IDLE (Run Module / F5), not only from the Windows .bat launcher.
if __name__ == '__main__':
    import sys
    from dependency_bootstrap import check_dependencies, ensure_dependencies
    if '--check-deps' in sys.argv:
        raise SystemExit(0 if check_dependencies() else 1)
    # Self-test exercises the standard-library/NumPy pipeline only; it should
    # never trigger a network installation just because E57/GUI packages are absent.
    if '--self-test' not in sys.argv and not ensure_dependencies():
        raise SystemExit('PointCloud Filter could not install its dependencies. See setup output above.')

import numpy as np

APP_NAME = 'PointCloud Filter'
APP_VERSION = '3.1.0'
CFG_PATH = Path.home() / '.pointcloud_filter_v3.json'
LAS_SCALE = 0.001
SUPPORTED_EXT = {'.las', '.laz', '.e57'}

@dataclass
class Options:
    input_dir: Path
    output_dir: Path
    scratch_dir: Path
    output_format: str = 'LAS'
    units: str = 'm'  # output units
    las_units: str = 'm'  # units of source LAS / LAZ coordinates
    grid_enabled: bool = True
    grid_size: float = 0.10
    range_enabled: bool = False
    range_min: float = 0.0
    range_max: float | None = None
    las_origin: tuple[float,float,float] | None = None  # source LAS XYZ units
    apply_e57_pose: bool = True
    recursive: bool = False
    chunk_rows: int = 50000

@dataclass
class Stats:
    attempted: int = 0
    valid: int = 0
    in_range: int = 0
    kept: int = 0

@dataclass(frozen=True)
class ScanJob:
    path: Path
    scan_index: int | None = None
    def label(self):
        return self.path.name + (f' (scan {self.scan_index+1})' if self.scan_index is not None else '')
    def output_name(self):
        return self.path.stem + (f'_scan_{self.scan_index+1:03d}' if self.scan_index is not None else '')

def metres_to_units(units: str) -> float:
    return {'m': 1.0, 'ft': 1.0/0.3048, 'usft': 3937.0/1200.0}[units]


def packed_color(red, green, blue):
    # E57 color bounds are often 0..255, LAS RGB is 0..65535.
    # Caller supplies normalized 0..255 channel arrays.
    r=np.clip(np.asarray(red),0,255).astype(np.uint32)
    g=np.clip(np.asarray(green),0,255).astype(np.uint32)
    b=np.clip(np.asarray(blue),0,255).astype(np.uint32)
    return r | (g << 8) | (b << 16)

class LasScan:
    def __init__(self, path: Path, opt: Options):
        try:
            import laspy
        except ImportError as exc:
            raise RuntimeError('LAS/LAZ reading requires laspy. Run pip install -r requirements.txt') from exc
        self.reader=laspy.open(str(path))
        self.total=int(self.reader.header.point_count)
        self.input_scale=metres_to_units(opt.units)/metres_to_units(opt.las_units)
        self.origin = (np.asarray(opt.las_origin, dtype=float) if opt.las_origin is not None else None)
        self.log_hint='LAS/LAZ: coordinate units are user-selected; enter scanner origin to enable range filtering.'
    def blocks(self, rows, cancel, tick):
        sofar=0
        try:
            for points in self.reader.chunk_iterator(rows):
                if cancel.is_set(): raise InterruptedError('Cancelled by user.')
                xyz=np.column_stack((np.asarray(points.x),np.asarray(points.y),np.asarray(points.z)))
                if {'red','green','blue'} <= set(points.point_format.dimension_names):
                    rawrgb=np.column_stack((np.asarray(points.red),np.asarray(points.green),np.asarray(points.blue)))
                    rgb=packed_color(*(np.rint(rawrgb[:,i]/257) for i in range(3)))
                elif 'intensity' in set(points.point_format.dimension_names):
                    gray=np.rint(np.asarray(points.intensity,dtype=np.float64)/257)
                    rgb=packed_color(gray, gray, gray)
                else:
                    rgb=np.zeros(len(xyz),dtype=np.uint32)
                sofar+=len(xyz)
                tick(min(1,sofar/max(1,self.total)))
                yield xyz, rgb, np.isfinite(xyz).all(axis=1)
        finally:
            pass
    def close(self): self.reader.close()


def _e57_color_limits(header):
    """Return normalized low/high RGB bounds from scan's optional colorLimits metadata."""
    defaults=[(0.,255.)]*3
    try:
        limits=header['colorLimits']
        return [(float(limits[f'color{c}Minimum'].value()),
                 float(limits[f'color{c}Maximum'].value())) for c in ('Red','Green','Blue')]
    except Exception:
        return defaults

class E57Scan:
    """One-scan-at-a-time low-level compressed-vector buffered E57 reader.
    read_scan() in pye57 allocates arrays for all points; do not use it here.
    """
    def __init__(self, path: Path, opt: Options, scan_index: int):
        try:
            import pye57
        except ImportError as exc:
            raise RuntimeError('E57 reading requires pye57. Run pip install -r requirements.txt') from exc
        self.e57=pye57.E57(str(path))
        self.index=scan_index
        self.header=self.e57.get_header(scan_index)
        self.total=int(self.header.point_count)
        fields=set(self.header.point_fields)
        if {'cartesianX','cartesianY','cartesianZ'} <= fields:
            self.coord_fields=['cartesianX','cartesianY','cartesianZ']
            self.invalid_field='cartesianInvalidState'
            self.spherical=False
        elif {'sphericalRange','sphericalAzimuth','sphericalElevation'} <= fields:
            self.coord_fields=['sphericalRange','sphericalAzimuth','sphericalElevation']
            self.invalid_field='sphericalInvalidState'
            self.spherical=True
        else:
            self.e57.close()
            raise RuntimeError('E57 scan has neither Cartesian nor spherical XYZ fields.')
        self.extra_fields=[k for k in ('colorRed','colorGreen','colorBlue') if k in fields]
        if 'intensity' in fields: self.extra_fields.append('intensity')
        if self.invalid_field in fields: self.extra_fields.append(self.invalid_field)
        self.translation=np.asarray(self.header.translation,dtype=np.float64)
        self.rotation=np.asarray(self.header.rotation_matrix,dtype=np.float64)
        if self.rotation.shape!=(3,3): raise RuntimeError('Invalid E57 scan pose matrix.')
        # ScanHeader supports the E57 node dictionary interface; the pose is optional.
        self.has_pose = self.header.has_pose()
        from station_model import inspect_station
        self.station_metadata = inspect_station(self.e57, scan_index)
        if self.station_metadata['position'] is None:
            self.has_pose = False
        else:
            self.translation = np.asarray(self.station_metadata['position'])
            self.rotation = np.asarray(self.station_metadata['rotation'])
        self.apply_pose=opt.apply_e57_pose and self.has_pose
        self.output_scale=metres_to_units(opt.units)
        self.color_limits=_e57_color_limits(self.header)
        self.log_hint=f'E57 scan {scan_index+1}: {self.total:,} points; pose='+('applied' if self.apply_pose else 'not applied (local XYZ)')
    def blocks(self, rows, cancel, tick):
        from pye57 import libe57
        fields=self.coord_fields+self.extra_fields
        # Float64 supports scaled integer and float E57 fields with conversion enabled.
        arrays={k: np.empty(rows,dtype=np.float64) for k in fields}
        buffers=libe57.VectorSourceDestBuffer()
        for key in fields:
            buffers.append(libe57.SourceDestBuffer(
                self.e57.image_file, key, arrays[key], rows, True, True))
        # Use the documented libE57 compressed-vector node rather than depending
        # on a convenience ScanHeader.points attribute.
        points = self.e57.image_file.root()['data3D'][self.index]['points']
        if not isinstance(points,libe57.CompressedVectorNode):
            points = libe57.CompressedVectorNode(points)
        reader=points.reader(buffers)
        done=0
        try:
            while True:
                if cancel.is_set(): raise InterruptedError('Cancelled by user.')
                n=int(reader.read())
                if n<=0: break
                if self.spherical:
                    ra=arrays['sphericalRange'][:n]; az=arrays['sphericalAzimuth'][:n]
                    el=arrays['sphericalElevation'][:n]
                    rc=ra*np.cos(el)
                    xyz=np.column_stack((rc*np.cos(az),rc*np.sin(az),ra*np.sin(el)))
                else:
                    xyz=np.column_stack(tuple(arrays[f][:n] for f in self.coord_fields))
                valid=np.isfinite(xyz).all(axis=1)
                if self.invalid_field in arrays:
                    valid &= arrays[self.invalid_field][:n] == 0
                if all(f in arrays for f in ('colorRed','colorGreen','colorBlue')):
                    channels=[]
                    for cname,(lo,hi) in zip(('colorRed','colorGreen','colorBlue'),self.color_limits):
                        vals=arrays[cname][:n]
                        if hi<=lo:
                            hi=65535. if np.nanmax(vals)>255 else 255.
                            lo=0.
                        # Some exporters omit colorLimits for 16-bit color.
                        elif hi == 255. and np.any(vals > 255.):
                            hi = 65535.
                        channels.append(np.rint(np.clip((vals-lo)*255.0/(hi-lo),0,255)))
                    rgb=packed_color(*channels)
                elif 'intensity' in arrays:
                    vals=arrays['intensity'][:n]
                    gray=np.rint(np.clip(vals*255 if np.nanmax(vals)<=1 else vals/257,0,255))
                    rgb=packed_color(gray,gray,gray)
                else:
                    rgb=np.zeros(n,dtype=np.uint32)
                done += n
                tick(min(1,done/max(1,self.total)))
                yield xyz, rgb, valid
        finally:
            reader.close()
    def close(self): self.e57.close()


def list_files(opt: Options) -> list[Path]:
    it=opt.input_dir.rglob('*') if opt.recursive else opt.input_dir.iterdir()
    return sorted((p for p in it if p.is_file() and p.suffix.lower() in SUPPORTED_EXT),
                  key=lambda p: str(p).lower())

def job_list(opt: Options) -> list[ScanJob]:
    jobs=[]
    for p in list_files(opt):
        if p.suffix.lower()=='.e57':
            try:
                import pye57
                with pye57.E57(str(p)) as e:
                    n=int(e.scan_count)
            except ImportError as exc:
                raise ValueError('To scan E57 files, install requirements.txt (pye57 missing).') from exc
            jobs.extend(ScanJob(p,i) for i in range(n))
        else:
            jobs.append(ScanJob(p))
    return jobs

UPSERT_SQL = '''
INSERT INTO cells (ci, cj, x, y, z, rgb) VALUES (?, ?, ?, ?, ?, ?)
ON CONFLICT(ci, cj) DO UPDATE SET
    x = excluded.x, y = excluded.y, z = excluded.z, rgb = excluded.rgb
WHERE excluded.z < cells.z
'''


class LowestGrid:
    """Disk-backed exact minimum-Z representative per horizontal XY cell."""

    def __init__(self, database: Path, cell: float):
        self.cell = cell
        self.con = sqlite3.connect(str(database))
        self.con.execute('PRAGMA journal_mode=DELETE')
        self.con.execute('PRAGMA synchronous=NORMAL')
        self.con.execute('PRAGMA temp_store=FILE')
        self.con.execute('PRAGMA cache_size=-32768')  # approx. 32 MiB DB page cache
        self.con.execute('''CREATE TABLE cells (
            ci INTEGER NOT NULL, cj INTEGER NOT NULL,
            x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
            rgb INTEGER NOT NULL, PRIMARY KEY (ci, cj)
        ) WITHOUT ROWID''')

    def add(self, xyz: np.ndarray, rgb: np.ndarray):
        if len(xyz) == 0:
            return
        ix = np.floor(xyz[:, 0] / self.cell).astype(np.int64)
        iy = np.floor(xyz[:, 1] / self.cell).astype(np.int64)
        order = np.lexsort((xyz[:, 2], iy, ix))
        si, sj = ix[order], iy[order]
        first = np.ones(len(order), dtype=bool)
        first[1:] = (si[1:] != si[:-1]) | (sj[1:] != sj[:-1])
        keep = order[first]
        records = (
            (int(ix[k]), int(iy[k]), float(xyz[k, 0]), float(xyz[k, 1]),
             float(xyz[k, 2]), int(rgb[k])) for k in keep
        )
        self.con.executemany(UPSERT_SQL, records)
        self.con.commit()

    def count(self) -> int:
        return int(self.con.execute('SELECT COUNT(*) FROM cells').fetchone()[0])

    def blocks(self, n: int = 50000) -> Iterator[tuple[np.ndarray, np.ndarray]]:
        cursor = self.con.execute('SELECT x, y, z, rgb FROM cells')
        while True:
            rows = cursor.fetchmany(n)
            if not rows:
                break
            arr = np.asarray(rows, dtype=np.float64)
            yield arr[:, :3].copy(), arr[:, 3].astype(np.uint32)

    def close(self):
        self.con.close()


def rgb_channels(packed: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """FARO mode 2: bits 0-7 R, 8-15 G, 16-23 B."""
    p = packed.astype(np.uint32, copy=False)
    return (p & 255).astype(np.uint16), ((p >> 8) & 255).astype(np.uint16), ((p >> 16) & 255).astype(np.uint16)


LAS12_HEADER_FMT = '<4sHHIHH8sBB32s32sHHHIIBHI5I3d3d6d'
LAS12_HEADER_BYTES = struct.calcsize(LAS12_HEADER_FMT)
LAS_POINT2_DTYPE = np.dtype([
    ('X', '<i4'), ('Y', '<i4'), ('Z', '<i4'),
    ('intensity', '<u2'), ('return_flags', 'u1'),
    ('classification', 'u1'), ('scan_angle', 'i1'),
    ('user_data', 'u1'), ('source_id', '<u2'),
    ('red', '<u2'), ('green', '<u2'), ('blue', '<u2'),
])
assert LAS12_HEADER_BYTES == 227
assert LAS_POINT2_DTYPE.itemsize == 26


class Las12Writer:
    """Incremental uncompressed LAS 1.2, point format 2 (XYZ + RGB).

    LAS 1.2 spec: 227-byte header and 26-byte point format 2. Avoids needing
    to hold all retained points in memory for the export phase.
    """

    def __init__(self, path: Path, first: np.ndarray | None):
        self.file = open(path, 'wb+')
        self.offset = np.asarray(first[0] if first is not None and len(first) else
                                 [0.0, 0.0, 0.0], dtype=np.float64)
        self.scale = np.array([LAS_SCALE] * 3, dtype=np.float64)
        self.n = 0
        self.min_xyz = np.array([np.inf] * 3)
        self.max_xyz = np.array([-np.inf] * 3)
        self.file.write(bytes(LAS12_HEADER_BYTES))

    def write(self, xyz: np.ndarray, r: np.ndarray, g: np.ndarray, b: np.ndarray):
        if self.n + len(xyz) >= 2**32:
            raise RuntimeError('LAS 1.2 supports at most 4,294,967,295 points/file.')
        quant = np.rint((xyz - self.offset) / self.scale)
        if not np.isfinite(quant).all() or np.any(np.abs(quant) > (2**31 - 2)):
            raise RuntimeError('Point exceeds LAS 32-bit XYZ range with current offset/scale.')
        out = np.zeros(len(xyz), dtype=LAS_POINT2_DTYPE)
        out['X'], out['Y'], out['Z'] = (quant[:, j].astype(np.int32) for j in range(3))
        out['red'], out['green'], out['blue'] = r * 257, g * 257, b * 257
        # Intensity is not carried by this RGB-only writer. Do not invent a measured return.
        out['intensity'] = 0
        out['return_flags'] = 9   # return 1, total returns 1; SCENE return metadata not read
        self.file.write(out.tobytes())
        self.n += len(xyz)
        self.min_xyz = np.minimum(self.min_xyz, xyz.min(axis=0))
        self.max_xyz = np.maximum(self.max_xyz, xyz.max(axis=0))

    def close(self):
        if self.file.closed:
            return
        today = datetime.date.today()
        bbox = (float(self.max_xyz[0]), float(self.min_xyz[0]),
                float(self.max_xyz[1]), float(self.min_xyz[1]),
                float(self.max_xyz[2]), float(self.min_xyz[2])) if self.n else (0.0,) * 6
        header = struct.pack(
            LAS12_HEADER_FMT,
            b'LASF', 0, 0, 0, 0, 0, bytes(8), 1, 2,
            b'PointCloud Filter'.ljust(32, b'\x00'),
            b'PointCloud v3.0'.ljust(32, b'\x00'),
            today.timetuple().tm_yday, today.year,
            LAS12_HEADER_BYTES, LAS12_HEADER_BYTES, 0,
            2, LAS_POINT2_DTYPE.itemsize, self.n,
            self.n, 0, 0, 0, 0,
            *[float(v) for v in self.scale],
            *[float(v) for v in self.offset],
            *bbox
        )
        self.file.seek(0)
        self.file.write(header)
        self.file.close()


class FileWriters:
    def __init__(self, destination: Path, kind: str, units: str):
        self.base = destination
        self.kind = kind
        self.units = units
        self.csv_handle = None
        self.csv_writer = None
        self.las_handle = None
        self.started = False
        self.temp_files: list[tuple[Path, Path]] = []
        self.kept = 0

    def _start(self, first: np.ndarray | None = None):
        if self.started:
            return
        self.started = True
        self.base.parent.mkdir(parents=True, exist_ok=True)
        if self.kind in ('CSV', 'BOTH'):
            final = self.base.with_suffix('.csv')
            part = final.with_name(final.stem + '.partial.csv')
            self.temp_files.append((part, final))
            self.csv_handle = open(part, 'w', encoding='utf-8', newline='', buffering=1024 * 1024)
            self.csv_writer = csv.writer(self.csv_handle)
            self.csv_writer.writerow(['x', 'y', 'z', 'red', 'green', 'blue'])
        if self.kind in ('LAS', 'BOTH'):
            final = self.base.with_suffix('.las')
            part = final.with_name(final.stem + '.partial.las')
            self.temp_files.append((part, final))
            # CRS is not declared: do not infer from exported coordinates.
            self.las_handle = Las12Writer(part, first)

    def write(self, xyz: np.ndarray, packed_rgb: np.ndarray):
        if len(xyz) == 0:
            return
        if not self.started:
            self._start(xyz)
        r, g, b = rgb_channels(packed_rgb)
        if self.csv_writer is not None:
            self.csv_writer.writerows(
                (f'{x:.6f}', f'{y:.6f}', f'{z:.6f}', int(rr), int(gg), int(bb))
                for (x, y, z), rr, gg, bb in zip(xyz, r, g, b)
            )
        if self.las_handle is not None:
            self.las_handle.write(xyz, r, g, b)
        self.kept += len(xyz)

    def finish(self, commit: bool) -> list[str]:
        try:
            if not self.started and commit:
                self._start(None)  # Create an empty, valid export.
            if self.csv_handle is not None:
                self.csv_handle.close()
                self.csv_handle = None
            if self.las_handle is not None:
                self.las_handle.close()
                self.las_handle = None
            if commit:
                for partial, final in self.temp_files:
                    os.replace(partial, final)
                return [str(final) for _, final in self.temp_files]
            return []
        finally:
            if not commit:
                for partial, _ in self.temp_files:
                    try:
                        partial.unlink(missing_ok=True)
                    except OSError:
                        pass


def unique_output_base(out_dir: Path, name: str, kind: str) -> Path:
    base = out_dir / f'{name}_filtered'
    extensions = ('.csv', '.las') if kind == 'BOTH' else ('.csv',) if kind == 'CSV' else ('.las',)
    i = 2
    while any(base.with_suffix(ext).exists() or
              base.with_name(base.stem + '.partial' + ext).exists() for ext in extensions):
        base = out_dir / f'{name}_filtered_{i}'
        i += 1
    return base



def validate(opt: Options) -> list[ScanJob]:
    if not opt.input_dir.is_dir():
        raise ValueError('Input directory does not exist.')
    if opt.output_format not in ('LAS','CSV','BOTH'):
        raise ValueError('Output format must be LAS, CSV or BOTH.')
    if opt.units not in ('m','ft','usft') or opt.las_units not in ('m','ft','usft'):
        raise ValueError('Units must be m, ft, or usft.')
    if opt.input_dir.resolve() == opt.output_dir.resolve():
        raise ValueError('Export directory must differ from the source directory.')
    if opt.recursive and opt.input_dir.resolve() in opt.output_dir.resolve().parents:
        raise ValueError('With recursive input discovery, the output folder must not be under the input folder (to avoid reprocessing output LAS files).')
    for target in (opt.output_dir, opt.scratch_dir):
        target.mkdir(parents=True, exist_ok=True)
        if not target.is_dir():
            raise ValueError(f'Cannot create directory: {target}')
    if opt.grid_enabled and (not math.isfinite(opt.grid_size) or opt.grid_size <= 0):
        raise ValueError('Minimum-Z grid cell size must be positive.')
    if opt.range_enabled:
        if not math.isfinite(opt.range_min) or opt.range_min < 0:
            raise ValueError('Minimum range must be zero or greater.')
        if opt.range_max is not None and (not math.isfinite(opt.range_max) or opt.range_max < opt.range_min):
            raise ValueError('Maximum range must be >= minimum range.')
        if any(f.suffix.lower() in ('.las','.laz') for f in list_files(opt)) and opt.las_origin is None:
            raise ValueError('LAS/LAZ range filtering needs a manually entered scanner X/Y/Z origin.')
    if opt.las_origin is not None and (len(opt.las_origin)!=3 or not np.isfinite(opt.las_origin).all()):
        raise ValueError('Scanner origin must contain three finite numbers.')
    if not 100 <= opt.chunk_rows <= 200000:
        raise ValueError('Points per batch must be between 100 and 200,000.')
    jobs=job_list(opt)
    if not jobs:
        raise ValueError('No E57, LAS or LAZ input scans found.')
    try:
        with tempfile.TemporaryFile(dir=str(opt.scratch_dir)):
            pass
        with tempfile.TemporaryFile(dir=str(opt.output_dir)):
            pass
    except OSError as e:
        raise ValueError('Cannot write to scratch or export folder.') from e
    return jobs


def process_one(job: ScanJob, opt: Options, stop: threading.Event,
                progress: Callable[[float],None], log: Callable[[str],None]) -> tuple[Stats,list[str]]:
    stats=Stats()
    scan=None
    grid=None
    writers=FileWriters(unique_output_base(opt.output_dir,job.output_name(),opt.output_format),opt.output_format,opt.units)
    committed=False
    try:
        with tempfile.TemporaryDirectory(prefix='pc_grid_',dir=str(opt.scratch_dir)) as work:
            if job.scan_index is None:
                scan=LasScan(job.path,opt)
            else:
                scan=E57Scan(job.path,opt,job.scan_index)
            log(f'  {scan.log_hint}')
            if opt.grid_enabled:
                grid=LowestGrid(Path(work)/'lowest_cells.sqlite',opt.grid_size)
            for local_xyz,rgb,valid_initial in scan.blocks(opt.chunk_rows,stop,progress):
                stats.attempted+=len(local_xyz)
                valid=np.asarray(valid_initial,dtype=bool).copy()
                valid &= np.isfinite(local_xyz).all(axis=1)
                stats.valid+=int(valid.sum())
                if opt.range_enabled:
                    # Compute distances in original scanner-local xyz BEFORE registration.
                    if job.scan_index is not None:
                        distances=np.linalg.norm(local_xyz,axis=1)*scan.output_scale
                    else:
                        distances=np.linalg.norm(local_xyz-scan.origin,axis=1)*scan.input_scale
                    valid &= distances >= opt.range_min
                    if opt.range_max is not None:
                        valid &= distances <= opt.range_max
                stats.in_range+=int(valid.sum())
                if not np.any(valid):
                    continue
                xyz=np.asarray(local_xyz[valid],dtype=np.float64).copy()
                selected=rgb[valid]
                if job.scan_index is not None:
                    if scan.apply_pose:
                        xyz=xyz@scan.rotation.T+scan.translation
                    xyz *= scan.output_scale
                else:
                    xyz *= scan.input_scale
                if grid is None:
                    writers.write(xyz,selected)
                else:
                    grid.add(xyz,selected)
            scan.close()
            scan=None
            if grid is not None:
                stats.kept=grid.count()
                log(f'  {stats.kept:,} grid cells with one minimum-Z point; exporting…')
                for xyz,rgb in grid.blocks(opt.chunk_rows):
                    if stop.is_set(): raise InterruptedError('Cancelled by user.')
                    writers.write(xyz,rgb)
                grid.close()
                grid=None
            else:
                stats.kept=writers.kept
            if stop.is_set(): raise InterruptedError('Cancelled by user.')
            outputs=writers.finish(True)
            committed=True
            return stats,outputs
    finally:
        if scan is not None: scan.close()
        if grid is not None: grid.close()
        if not committed: writers.finish(False)


def run_batch(opt: Options, log: Callable[[str],None], tick: Callable[[float],None],
              stop: threading.Event) -> tuple[list[list], bool]:
    jobs=validate(opt)
    log(f'{len(jobs)} input scan(s) queued; scratch folder: {opt.scratch_dir}')
    report=[]; cancelled=False
    for i,job in enumerate(jobs):
        if stop.is_set(): cancelled=True; break
        log(f'[{i+1}/{len(jobs)}] {job.label()}')
        start=time.monotonic()
        try:
            stats,files=process_one(job,opt,stop,lambda f:tick((i+f)/len(jobs)),log)
            elapsed=time.monotonic()-start
            log(f'  DONE: valid={stats.valid:,}; in range={stats.in_range:,}; retained={stats.kept:,} ({elapsed:.1f}s)')
            for f in files: log('  Saved: '+f)
            report.append([str(job.path),job.scan_index if job.scan_index is not None else '',
                          'OK',stats.attempted,stats.valid,stats.in_range,stats.kept,
                          f'{elapsed:.1f}','; '.join(files),''])
        except InterruptedError:
            cancelled=True
            log('  CANCELLED: incomplete exports and scratch files removed')
            break
        except Exception as e:
            log('  ERROR: '+str(e))
            report.append([str(job.path),job.scan_index if job.scan_index is not None else '',
                           'ERROR','','','','',f'{time.monotonic()-start:.1f}','',str(e)])
        tick((i+1)/len(jobs))
    if report:
        report_file=opt.output_dir/('processing_report_'+time.strftime('%Y%m%d_%H%M%S')+'.csv')
        with open(report_file,'w',newline='',encoding='utf-8') as fh:
            wr=csv.writer(fh)
            wr.writerow(['input_file','scan_index_zero_based','status','attempted','valid',
                         'after_range','retained','elapsed_seconds','export_files','error'])
            wr.writerows(report)
        log('Report: '+str(report_file))
    return report,cancelled


def scan_source_identity(job: ScanJob, opt: Options):
    """Fast source-file identity + import-setting signature; not a content hash."""
    import hashlib
    from database_backend import ScanSource
    resolved=job.path.resolve()
    st=resolved.stat()
    scan_index=job.scan_index if job.scan_index is not None else -1
    key=str(resolved).casefold()+f'::{scan_index}'
    import_settings={'source_path':str(resolved), 'size':st.st_size,
                     'mtime_ns':st.st_mtime_ns, 'scan':scan_index,
                     'apply_e57_pose':opt.apply_e57_pose if scan_index>=0 else None,
                     'las_input_units':opt.las_units if scan_index<0 else None,
                     'las_origin':opt.las_origin if scan_index<0 else None}
    signature=hashlib.sha256(json.dumps(import_settings,sort_keys=True).encode()).hexdigest()
    return key,signature


def db_config_check(opt: Options, database_path: Path, mem_gb: int,
                    *, importing: bool = False):
    if not 1 <= mem_gb <= 256:
        raise ValueError('Set the database memory budget to 1–256 GB.')
    if not 100 <= opt.chunk_rows <= 200000:
        raise ValueError('Points per batch must be 100–200,000.')
    if not opt.scratch_dir.is_dir(): opt.scratch_dir.mkdir(parents=True,exist_ok=True)
    if database_path.suffix.lower() != '.duckdb':
        raise ValueError('The database path should end in .duckdb.')
    if database_path.is_dir():
        raise ValueError('The selected database path is a directory.')
    if opt.units not in ('m','ft','usft') or opt.las_units not in ('m','ft','usft'):
        raise ValueError('Coordinate units must be m, ft, or usft.')
    if opt.las_origin is not None and not np.isfinite(opt.las_origin).all():
        raise ValueError('Scanner-origin coordinates must be finite.')
    if importing and not opt.input_dir.is_dir():
        raise ValueError('Select an existing input directory to import.')
    if importing and opt.recursive and opt.output_dir.is_dir() and opt.output_dir.resolve().is_relative_to(opt.input_dir.resolve()):
        raise ValueError('With recursive input import, export directory must not be inside the input tree.')
    if not importing:
        if opt.grid_enabled and (not math.isfinite(opt.grid_size) or opt.grid_size<=0):
            raise ValueError('Grid cell size must be positive.')
        if opt.range_enabled and (opt.range_min<0 or not math.isfinite(opt.range_min) or
                (opt.range_max is not None and (not math.isfinite(opt.range_max) or
                    opt.range_max < opt.range_min))):
            raise ValueError('Invalid scanner range limits.')
        if opt.output_format not in ('LAS','CSV','BOTH'):
            raise ValueError('Choose LAS, CSV, or BOTH export.')
        opt.output_dir.mkdir(parents=True,exist_ok=True)
    try:
        with tempfile.TemporaryFile(dir=opt.scratch_dir): pass
    except OSError as e:
        raise ValueError('Cannot write to external scratch folder.') from e


def import_to_database(opt: Options, db_path: Path, memory_gb: int,
                       force: bool, log: Callable[[str],None],
                       tick: Callable[[float],None], stop: threading.Event):
    """FLS-free, resumable-by-scan E57/LAS import into persistent database.

    An existing READY scan is never removed before the new version is complete.
    """
    from database_backend import open_database, ScanSource
    db_config_check(opt,db_path,memory_gb,importing=True)
    jobs=job_list(opt)
    if not jobs: raise ValueError('No E57, LAS or LAZ input scans found.')
    results=[]
    log(f'Found {len(jobs)} scan(s); database: {db_path}')
    with open_database('DuckDB',db_path,opt.scratch_dir,memory_gb) as db:
        for i,job in enumerate(jobs):
            if stop.is_set(): return results,True
            label=job.label()
            start=time.monotonic()
            scan=None
            new_scan_id=None
            attempted=0
            valid_count=0
            try:
                key,signature=scan_source_identity(job,opt)
                current=db.find_current(key)
                if not force and current is not None and current.source_signature == signature:
                    log(f'[{i+1}/{len(jobs)}] SKIP unchanged: {label}  ({current.point_count:,} DB points)')
                    results.append((label,'SKIPPED',current.point_count))
                    tick((i+1)/len(jobs))
                    continue
                log(f'[{i+1}/{len(jobs)}] Importing: {label}')
                if job.scan_index is None:
                    scan=LasScan(job.path,opt)
                    coordinate_frame='LAS_XYZ'
                    pose_applied=False
                    range_known=opt.las_origin is not None
                else:
                    scan=E57Scan(job.path,opt,job.scan_index)
                    coordinate_frame='E57_POSED' if scan.apply_pose else 'E57_SCAN_LOCAL'
                    pose_applied=bool(scan.apply_pose)
                    range_known=True  # E57 local XYZ is relative to scanner origin
                    if not pose_applied:
                        log('  NOTE: E57 pose absent/disabled: storing scan-local coordinates.')
                log('  '+scan.log_hint)
                meta=ScanSource(str(job.path.resolve()),key,
                                job.scan_index if job.scan_index is not None else -1,
                                job.path.suffix.lower().lstrip('.'),signature,
                                coordinate_frame,pose_applied,range_known)
                new_scan_id=db.begin_scan(meta)
                if job.scan_index is None and opt.las_origin is not None:
                    db.save_station(new_scan_id,dict(name=job.path.stem,
                        position=(np.asarray(opt.las_origin)/metres_to_units(opt.las_units)).tolist(),
                        rotation=np.eye(3).tolist(),images=[],version=1))
                if hasattr(scan,'station_metadata'):
                    db.save_station(new_scan_id,scan.station_metadata)
                    images=scan.station_metadata['images']
                    log(f'  Station metadata: pose={scan.station_metadata["position"] is not None}; associated images={len(images)}')
                for raw_xyz,rgb,initial_valid in scan.blocks(
                        opt.chunk_rows,stop,lambda f:tick((i+f)/len(jobs))):
                    if stop.is_set(): raise InterruptedError('User cancelled import.')
                    attempted+=len(raw_xyz)
                    valid=np.asarray(initial_valid,dtype=bool).copy()
                    valid &= np.isfinite(raw_xyz).all(axis=1)
                    if not np.any(valid): continue
                    xyz=np.asarray(raw_xyz[valid],dtype=np.float64).copy()
                    rgb_keep=np.asarray(rgb[valid],dtype=np.uint32)
                    if job.scan_index is not None:
                        range_m=np.linalg.norm(xyz,axis=1)
                        if pose_applied:
                            xyz=xyz@scan.rotation.T+scan.translation
                    else:
                        # LAS source coordinates are in user-declared units.
                        if range_known:
                            range_m=np.linalg.norm(xyz-np.asarray(opt.las_origin),axis=1) / metres_to_units(opt.las_units)
                        else:
                            range_m=None
                        xyz/=metres_to_units(opt.las_units)
                    valid2=np.isfinite(xyz).all(axis=1)
                    if range_m is not None:
                        valid2 &= np.isfinite(range_m)
                    xyz=xyz[valid2]
                    rgb_keep=rgb_keep[valid2]
                    if range_m is not None: range_m=range_m[valid2]
                    if not len(xyz): continue
                    db.append_points(new_scan_id,xyz,rgb_keep,range_m)
                    valid_count+=len(xyz)
                if stop.is_set(): raise InterruptedError('User cancelled import.')
                scan.close();scan=None
                db.complete_scan(new_scan_id,attempted,valid_count)
                new_scan_id=None
                log(f'  IMPORTED: {valid_count:,} valid points from {attempted:,} attempted ({time.monotonic()-start:.1f}s).')
                results.append((label,'IMPORTED',valid_count))
            except InterruptedError:
                if new_scan_id is not None: db.abort_scan(new_scan_id)
                log(f'  CANCELLED: incomplete database scan {label} discarded.')
                return results,True
            except Exception as exc:
                if new_scan_id is not None: db.abort_scan(new_scan_id)
                log('  IMPORT ERROR: '+str(exc))
                results.append((label,'ERROR',str(exc)))
            finally:
                if scan is not None: scan.close()
            tick((i+1)/len(jobs))
        log(f'Database contains {len(db.list_scans())} current completed scan(s).')
    return results,False


def export_from_database(opt: Options, db_path: Path, memory_gb: int,
                         combined: bool, log: Callable[[str],None],
                         tick: Callable[[float],None], stop: threading.Event):
    """Repeatable SQL filtering against stored original-resolution points."""
    from database_backend import open_database
    db_config_check(opt,db_path,memory_gb,importing=False)
    if not db_path.is_file():
        raise ValueError('Database does not exist yet. Use Import / Update DB first.')
    reports=[]
    out_scale=metres_to_units(opt.units)
    cell_m=(opt.grid_size/out_scale) if opt.grid_enabled else None
    min_m=(opt.range_min/out_scale) if opt.range_enabled else None
    max_m=(opt.range_max/out_scale) if opt.range_enabled and opt.range_max is not None else None
    with open_database('DuckDB',db_path,opt.scratch_dir,memory_gb) as db:
        scans=db.list_scans()
        if not scans: raise ValueError('Database has no completed scan imports.')
        groups=[scans] if combined else [[s] for s in scans]
        if combined:
            log('COMBINED thinning: you confirmed all source scans share the SAME horizontal and vertical reference.')
        log(f'Querying {len(scans)} imported scans, {len(groups)} output group(s). Original E57/LAS files are not reread.')
        for i,group in enumerate(groups):
            if stop.is_set(): return reports,True
            name='combined_project' if combined else group[0].output_name
            outbase=unique_output_base(opt.output_dir,name,opt.output_format)
            writer=FileWriters(outbase,opt.output_format,opt.units)
            commit=False
            start=time.monotonic()
            total_input=sum(s.point_count for s in group)
            try:
                log(f'[{i+1}/{len(groups)}] Filtering: '+ ('all imported scans' if combined else group[0].label))
                if opt.range_enabled and any(not s.scanner_origin_known for s in group):
                    raise ValueError('Some LAS scans lack scanner XYZ origin. Reimport with origin before range filtering.')
                for xyz_m,rgb in db.iter_filtered([s.scan_id for s in group],
                                                  cell_m,min_m,max_m,opt.chunk_rows):
                    if stop.is_set(): raise InterruptedError('Cancelled by user.')
                    writer.write(xyz_m*out_scale,rgb)
                if stop.is_set(): raise InterruptedError('Cancelled by user.')
                files=writer.finish(True)
                commit=True
                log(f'  RETAINED {writer.kept:,} / {total_input:,} database points in {time.monotonic()-start:.1f}s.')
                for f in files: log('  Exported: '+f)
                reports.append((name,'OK',total_input,writer.kept,'; '.join(files),'',f'{time.monotonic()-start:.1f}'))
            except InterruptedError:
                log('  CANCELLED: incomplete output removed.')
                return reports,True
            except Exception as exc:
                log('  EXPORT ERROR: '+str(exc))
                reports.append((name,'ERROR',total_input,'','',str(exc),f'{time.monotonic()-start:.1f}'))
            finally:
                if not commit: writer.finish(False)
            tick((i+1)/len(groups))
    if reports:
        report_path=opt.output_dir/('database_export_report_'+time.strftime('%Y%m%d_%H%M%S')+'.csv')
        with open(report_path,'w',encoding='utf-8',newline='') as file:
            w=csv.writer(file)
            w.writerow(['selection','status','database_points','exported_points','output_files','error','seconds'])
            w.writerows(reports)
        log('Report: '+str(report_path))
    return reports,False

def gui():
    try:
        import customtkinter as ctk
        from tkinter import filedialog,messagebox
    except ImportError as exc:
        raise RuntimeError('Install GUI dependencies with: pip install -r requirements.txt') from exc
    ctk.set_appearance_mode('dark')
    ctk.set_default_color_theme('blue')

    class App(ctk.CTk):
        def __init__(self):
            super().__init__()
            self.title(f'PointCloud Filter {APP_VERSION} | Scan Map + Bubble Viewer')
            self.geometry('980x920')
            self.minsize(840,730)
            self.events=queue.Queue()
            self.stop=threading.Event()
            self.worker=None
            try:
                self.settings=json.loads(CFG_PATH.read_text(encoding='utf-8'))
            except (ValueError,OSError):
                try:
                    self.settings=json.loads((Path.home()/'.pointcloud_filter_v2.json').read_text(encoding='utf-8'))
                except (ValueError,OSError):
                    self.settings={}
            self.v={}
            self._build()
            self.after(120,self._poll)
            self.protocol('WM_DELETE_WINDOW',self._close)

        def var(self,name,default=''):
            v=ctk.StringVar(value=str(self.settings.get(name,default)))
            self.v[name]=v
            return v

        def folder(self,key):
            value=filedialog.askdirectory(initialdir=self.v[key].get() or str(Path.home()))
            if value: self.v[key].set(value)

        def path_row(self,parent,label,key,default,row):
            ctk.CTkLabel(parent,text=label,width=150,anchor='w').grid(row=row,column=0,sticky='w',padx=(14,4),pady=5)
            ctk.CTkEntry(parent,textvariable=self.var(key,default),height=30).grid(row=row,column=1,sticky='ew',padx=4,pady=5)
            ctk.CTkButton(parent,text='Browse',width=75,command=lambda:self.folder(key)).grid(row=row,column=2,padx=(3,14),pady=5)

        def select_database(self):
            current=self.v['database_path'].get().strip()
            value=filedialog.asksaveasfilename(
                title='Choose a local DuckDB project file',
                initialdir=str(Path(current).parent) if current else str(Path.home()),
                initialfile=Path(current).name if current else 'project.duckdb',
                defaultextension='.duckdb',filetypes=[('DuckDB project','*.duckdb')])
            if value: self.v['database_path'].set(value)

        def _build(self):
            outer=ctk.CTkFrame(self,fg_color='transparent')
            outer.pack(fill='both',expand=True,padx=15,pady=12)
            root=ctk.CTkScrollableFrame(outer,fg_color='transparent')
            root.pack(fill='both',expand=True)
            ctk.CTkLabel(root,text='POINTCLOUD  /  FILTER',font=ctk.CTkFont(size=22,weight='bold'),anchor='w').pack(fill='x')
            ctk.CTkLabel(root,text='E57 / LAS / LAZ  →  DuckDB on external SSD  →  LAS / CSV   |   SDK-free',anchor='w',text_color='gray65').pack(fill='x',pady=(0,10))
            paths=ctk.CTkFrame(root,corner_radius=10)
            paths.pack(fill='x',pady=(0,8))
            paths.grid_columnconfigure(1,weight=1)
            self.path_row(paths,'Input directory','input_dir','',0)
            self.path_row(paths,'Export directory','output_dir','',1)
            self.path_row(paths,'Scratch / external SSD','scratch_dir',tempfile.gettempdir(),2)
            self.path_row(paths,'Panorama cache (optional)','cache_dir','',4)
            dbpanel=ctk.CTkFrame(root,corner_radius=10)
            dbpanel.pack(fill='x',pady=(0,8))
            dbpanel.grid_columnconfigure(1,weight=1)
            ctk.CTkLabel(dbpanel,text='PERSISTENT DATABASE  /  DuckDB',font=ctk.CTkFont(weight='bold'),anchor='w').grid(row=0,column=0,columnspan=3,sticky='w',padx=14,pady=(10,4))
            ctk.CTkLabel(dbpanel,text='Project DB (.duckdb)',width=150,anchor='w').grid(row=1,column=0,sticky='w',padx=(14,4),pady=5)
            ctk.CTkEntry(dbpanel,textvariable=self.var('database_path',''),height=30).grid(row=1,column=1,sticky='ew',padx=4,pady=5)
            ctk.CTkButton(dbpanel,text='Browse',width=75,command=self.select_database).grid(row=1,column=2,padx=(3,14),pady=5)
            settings_row=ctk.CTkFrame(dbpanel,fg_color='transparent');settings_row.grid(row=2,column=0,columnspan=3,sticky='ew',padx=14,pady=(5,9))
            ctk.CTkLabel(settings_row,text='RAM budget (GB)',anchor='w').pack(side='left')
            ctk.CTkEntry(settings_row,textvariable=self.var('memory_gb','4'),width=58).pack(side='left',padx=(8,18))
            self.combine=ctk.BooleanVar(value=bool(self.settings.get('combined',False)))
            ctk.CTkCheckBox(settings_row,text='Combine ALL scans (only when aligned)',variable=self.combine).pack(side='left',padx=(0,12))
            self.force=ctk.BooleanVar(value=bool(self.settings.get('force_reimport',False)))
            ctk.CTkCheckBox(settings_row,text='Force reimport',variable=self.force).pack(side='left')
            ctk.CTkLabel(dbpanel,text='The DB stores complete original-resolution points in meters; filtering does not modify them.',
                         text_color='gray65',font=ctk.CTkFont(size=11),anchor='w').grid(row=3,column=0,columnspan=3,sticky='ew',padx=14,pady=(0,9))
            ctk.CTkButton(dbpanel,text='Scan Map + Bubble Viewer',command=self.open_scan_browser).grid(row=4,column=0,columnspan=3,padx=14,pady=(0,10),sticky='ew')
            self.recurse=ctk.BooleanVar(value=bool(self.settings.get('recursive',False)))
            ctk.CTkCheckBox(paths,text='Include subfolders',variable=self.recurse).grid(row=3,column=1,sticky='w',padx=4,pady=(4,12))
            panels=ctk.CTkFrame(root,fg_color='transparent')
            panels.pack(fill='x',pady=(0,8))
            panels.grid_columnconfigure((0,1),weight=1,uniform='col')
            left=ctk.CTkFrame(panels);left.grid(row=0,column=0,sticky='nsew',padx=(0,4))
            right=ctk.CTkFrame(panels);right.grid(row=0,column=1,sticky='nsew',padx=(4,0))
            ctk.CTkLabel(left,text='FILTERS',font=ctk.CTkFont(weight='bold')).pack(anchor='w',padx=12,pady=(10,7))
            self.grid_on=ctk.BooleanVar(value=bool(self.settings.get('grid_enabled',True)))
            ctk.CTkCheckBox(left,text='Lowest Z within XY grid cell',variable=self.grid_on).pack(anchor='w',padx=12,pady=3)
            r=ctk.CTkFrame(left,fg_color='transparent');r.pack(fill='x',padx=12,pady=5)
            ctk.CTkLabel(r,text='Cell size',width=102,anchor='w').pack(side='left')
            ctk.CTkEntry(r,textvariable=self.var('grid_size','0.10'),width=115).pack(side='left')
            self.range_on=ctk.BooleanVar(value=bool(self.settings.get('range_enabled',False)))
            ctk.CTkCheckBox(left,text='Scanner range (3D)',variable=self.range_on).pack(anchor='w',padx=12,pady=(6,3))
            r=ctk.CTkFrame(left,fg_color='transparent');r.pack(fill='x',padx=12,pady=(3,8))
            ctk.CTkLabel(r,text='Min',width=35).pack(side='left')
            ctk.CTkEntry(r,textvariable=self.var('range_min','0'),width=90).pack(side='left')
            ctk.CTkLabel(r,text='Max',width=40).pack(side='left',padx=(8,0))
            ctk.CTkEntry(r,textvariable=self.var('range_max',''),width=100,placeholder_text='unlimited').pack(side='left')
            ctk.CTkLabel(right,text='OUTPUT / COORDINATES',font=ctk.CTkFont(weight='bold')).pack(anchor='w',padx=12,pady=(10,7))
            r=ctk.CTkFrame(right,fg_color='transparent');r.pack(fill='x',padx=12,pady=4)
            ctk.CTkLabel(r,text='Export',width=98,anchor='w').pack(side='left')
            self.fmt=ctk.StringVar(value=self.settings.get('output_format','LAS'))
            ctk.CTkOptionMenu(r,values=['LAS','CSV','BOTH'],variable=self.fmt,width=140).pack(side='left')
            r=ctk.CTkFrame(right,fg_color='transparent');r.pack(fill='x',padx=12,pady=4)
            ctk.CTkLabel(r,text='Output units',width=98,anchor='w').pack(side='left')
            self.units=ctk.StringVar(value=self.settings.get('units','m'))
            ctk.CTkOptionMenu(r,values=['m','ft','usft'],variable=self.units,width=140).pack(side='left')
            r=ctk.CTkFrame(right,fg_color='transparent');r.pack(fill='x',padx=12,pady=4)
            ctk.CTkLabel(r,text='LAS input units',width=98,anchor='w').pack(side='left')
            self.las_units=ctk.StringVar(value=self.settings.get('las_units','m'))
            ctk.CTkOptionMenu(r,values=['m','ft','usft'],variable=self.las_units,width=140).pack(side='left')
            self.pose=ctk.BooleanVar(value=bool(self.settings.get('apply_e57_pose',True)))
            ctk.CTkCheckBox(right,text='Apply E57 scan pose (registered XYZ)',variable=self.pose).pack(anchor='w',padx=12,pady=(10,8))
            params=ctk.CTkFrame(root);params.pack(fill='x',pady=(0,8))
            ctk.CTkLabel(params,text='LAS / LAZ scanner origin XYZ',anchor='w').grid(row=0,column=0,padx=(12,7),pady=8)
            for i,k in enumerate(('origin_x','origin_y','origin_z')):
                ctk.CTkEntry(params,textvariable=self.var(k,''),width=108,placeholder_text=('X','Y','Z')[i]).grid(row=0,column=i+1,padx=4,pady=8)
            ctk.CTkLabel(params,text='Points / batch',anchor='w').grid(row=0,column=4,padx=(10,3))
            ctk.CTkEntry(params,textvariable=self.var('chunk_rows','50000'),width=83).grid(row=0,column=5,padx=(4,12))
            ctk.CTkLabel(root,text='E57 poses and coordinates are in meters. Range is measured before pose. '
                         'Lowest-Z thinning assumes Z is vertical. Use an SSD scratch drive for very large grids.',
                         text_color='#d9af6a',font=ctk.CTkFont(size=11),anchor='w',justify='left',wraplength=850).pack(fill='x',pady=(0,9))
            actions=ctk.CTkFrame(root,fg_color='transparent');actions.pack(fill='x')
            self.import_btn=ctk.CTkButton(actions,text='1  Import / Update DB',height=36,width=174,command=lambda:self._launch('import'))
            self.import_btn.pack(side='left')
            self.filter_btn=ctk.CTkButton(actions,text='2  Filter DB → Export',height=36,width=174,command=lambda:self._launch('export'))
            self.filter_btn.pack(side='left',padx=7)
            self.start_btn=ctk.CTkButton(actions,text='Direct (no DB)',height=36,width=142,fg_color='#42526a',command=lambda:self._launch('direct'))
            self.start_btn.pack(side='left')
            self.cancel_btn=ctk.CTkButton(actions,text='Cancel',height=36,width=90,state='disabled',fg_color='gray35',command=self._cancel)
            self.cancel_btn.pack(side='left',padx=8)
            self.status=ctk.CTkLabel(actions,text='Ready',anchor='e',text_color='gray65');self.status.pack(side='right')
            self.bar=ctk.CTkProgressBar(root,height=12);self.bar.pack(fill='x',pady=(10,8));self.bar.set(0)
            ctk.CTkLabel(outer,text='PROCESSING LOG',font=ctk.CTkFont(size=12,weight='bold'),anchor='w').pack(fill='x',pady=(10,4))
            self.log=ctk.CTkTextbox(outer,font=('Consolas',12),wrap='word',height=165)
            self.log.pack(fill='x')
            self.log.insert('end','1) Select E57 input + DuckDB file on SSD. Import once.  2) Filter DB repeatedly.\n'
                                  'The Direct button uses the legacy single-pass mode. Input files are never modified.\n')
            self.log.configure(state='disabled')

        def open_scan_browser(self):
            if self.worker is not None and self.worker.is_alive():
                messagebox.showinfo('Processing','Wait for processing to finish or cancel it first.')
                return
            try:
                from scan_viewer import ScanBrowser
                path=Path(self.v['database_path'].get().strip()).expanduser()
                if not path.is_file(): raise ValueError('Select an existing imported project database.')
                window=ScanBrowser(self,path,Path(self.v['scratch_dir'].get()).expanduser(),int(self.v['memory_gb'].get()),
                                   Path(self.v['cache_dir'].get()).expanduser() if self.v['cache_dir'].get().strip() else None)
                window.grab_set()
                self.wait_window(window)
            except Exception as exc:
                messagebox.showerror('Scan browser',str(exc))

        def get_filter_options(self):
            vmax=self.v['range_max'].get().strip()
            org=[self.v[k].get().strip() for k in ('origin_x','origin_y','origin_z')]
            if any(org) and not all(org):
                raise ValueError('Enter all three scanner-origin coordinates or leave all blank.')
            origin=tuple(float(v) for v in org) if all(org) else None
            return Options(input_dir=Path(self.v['input_dir'].get()).expanduser(),
                           output_dir=Path(self.v['output_dir'].get()).expanduser(),
                           scratch_dir=Path(self.v['scratch_dir'].get()).expanduser(),
                           output_format=self.fmt.get(),units=self.units.get(),las_units=self.las_units.get(),
                           grid_enabled=self.grid_on.get(),grid_size=float(self.v['grid_size'].get()),
                           range_enabled=self.range_on.get(),range_min=float(self.v['range_min'].get()),
                           range_max=float(vmax) if vmax else None,las_origin=origin,
                           apply_e57_pose=self.pose.get(),recursive=self.recurse.get(),
                           chunk_rows=int(self.v['chunk_rows'].get()))

        def _save(self):
            payload={k:v.get() for k,v in self.v.items()}
            payload.update(output_format=self.fmt.get(),units=self.units.get(),las_units=self.las_units.get(),
                           grid_enabled=self.grid_on.get(),range_enabled=self.range_on.get(),
                           apply_e57_pose=self.pose.get(),recursive=self.recurse.get(),
                           combined=self.combine.get(),force_reimport=self.force.get())
            try: CFG_PATH.write_text(json.dumps(payload,indent=2),encoding='utf-8')
            except OSError: pass

        def _launch(self, mode):
            try:
                opt=self.get_filter_options()
                if not self.v['scratch_dir'].get().strip():
                    raise ValueError('Select scratch / external SSD directory.')
                opt.scratch_dir.mkdir(parents=True,exist_ok=True)
                if mode in ('import','direct') and (not self.v['input_dir'].get().strip() or not opt.input_dir.is_dir()):
                    raise ValueError('Select an existing input directory.')
                if mode in ('export','direct'):
                    if not self.v['output_dir'].get().strip():
                        raise ValueError('Select an export directory.')
                    opt.output_dir.mkdir(parents=True,exist_ok=True)
                if mode=='direct' and opt.input_dir.resolve()==opt.output_dir.resolve():
                    raise ValueError('Input and export folders must differ.')
                database_path=Path(self.v['database_path'].get().strip()).expanduser()
                memory=int(self.v['memory_gb'].get())
                if mode!='direct' and not self.v['database_path'].get().strip():
                    raise ValueError('Select a .duckdb file, preferably on an external SSD.')
                if mode!='direct': db_config_check(opt,database_path,memory,importing=(mode=='import'))
                if mode=='export' and self.combine.get():
                    if not messagebox.askyesno('Combine scans — verify coordinate systems',
                            'Combining assumes ALL database scans are registered to the SAME horizontal and vertical coordinate system. '
                            'This cannot be verified automatically. Combine anyway?'):
                        return
                available=shutil.disk_usage(opt.scratch_dir).free/(1024**3)
                if mode=='import':
                    db_parent=database_path.parent
                    db_parent.mkdir(parents=True,exist_ok=True)
                    db_free=shutil.disk_usage(db_parent).free/(1024**3)
                    if db_free<2 and not messagebox.askyesno('Low database space',
                            f'The database drive has only {db_free:.2f} GiB free. Continue?'):
                        return
                if available<2 and not messagebox.askyesno('Low scratch space',
                        f'The scratch drive has only {available:.2f} GiB free. Continue?'):
                    return
            except (OSError,ValueError) as e:
                messagebox.showerror('Configuration error',str(e));return
            self._save();self.stop.clear();self.bar.set(0)
            for btn in (self.start_btn,self.import_btn,self.filter_btn):
                btn.configure(state='disabled')
            self.cancel_btn.configure(state='normal')
            self.worker=threading.Thread(target=self._worker,args=(mode,opt,database_path,memory,self.force.get(),self.combine.get()),daemon=True)
            self.worker.start()

        def _worker(self,mode,opt,database_path,memory,force,combine):
            try:
                log=lambda s:self.events.put(('log',s))
                tick=lambda f:self.events.put(('progress',f))
                if mode=='import':
                    self.events.put(('status','Importing E57 / LAS…'))
                    records,cancelled=import_to_database(opt,database_path,memory,force,log,tick,self.stop)
                    errors=sum(1 for r in records if r[1]=='ERROR')
                elif mode=='export':
                    self.events.put(('status','Filtering from DuckDB…'))
                    records,cancelled=export_from_database(opt,database_path,memory,combine,log,tick,self.stop)
                    errors=sum(1 for r in records if r[1]=='ERROR')
                else:
                    self.events.put(('status','Direct file processing…'))
                    records,cancelled=run_batch(opt,log,tick,self.stop)
                    errors=sum(1 for r in records if r[2]=='ERROR')
                message='Cancelled' if cancelled else f'Finished with {errors} error(s)' if errors else 'Complete'
                self.events.put(('done',message))
            except Exception:
                self.events.put(('fatal',traceback.format_exc()))

        def _cancel(self):
            self.stop.set();self.status.configure(text='Cancelling…');self.cancel_btn.configure(state='disabled')

        def _poll(self):
            try:
                while True:
                    typ,val=self.events.get_nowait()
                    if typ=='log':
                        self.log.configure(state='normal');self.log.insert('end',val+'\n')
                        self.log.see('end');self.log.configure(state='disabled')
                    elif typ=='status':self.status.configure(text=val)
                    elif typ=='progress':self.bar.set(float(val))
                    elif typ in ('done','fatal'):
                        [btn.configure(state='normal') for btn in (self.start_btn,self.import_btn,self.filter_btn)];self.cancel_btn.configure(state='disabled')
                        self.status.configure(text=val if typ=='done' else 'Error')
                        self.log.configure(state='normal');self.log.insert('end',val+'\n')
                        self.log.see('end');self.log.configure(state='disabled')
                        if typ=='fatal': messagebox.showerror('Processing error',val[-2500:])
            except queue.Empty:pass
            self.after(120,self._poll)

        def _close(self):
            if self.worker is not None and self.worker.is_alive():
                messagebox.showwarning('Running','Click Cancel and allow processing to stop before closing.')
                return
            self._save();self.destroy()
    App().mainloop()


def self_test():
    with tempfile.TemporaryDirectory() as folder:
        p=Path(folder)
        grid=LowestGrid(p/'cells.db',1.0)
        grid.add(np.array([[0.1,0.1,3.0],[0.2,0.2,1.0],[1.5,0.0,4.0]]),
                 np.array([0x102030,0x203040,0x304050],dtype=np.uint32))
        assert grid.count()==2
        xyz,rgb=next(grid.blocks())
        assert sorted(xyz[:,2])==[1.0,4.0]
        grid.close()
        w=FileWriters(p/'test','BOTH','m');w.write(xyz,rgb)
        written=w.finish(True)
        assert len(written)==2
        assert (p/'test.las').read_bytes()[:4]==b'LASF'
        assert (p/'test.csv').read_text().splitlines()[0]=='x,y,z,red,green,blue'
    print('PASS: SQLite minimum-Z filtering, LAS/CSV writers, scratch cleanup')
    print('NOTICE: E57/LAZ reader requires external dependencies and real input files to test.')

if __name__=='__main__':
    import sys
    if '--self-test' in sys.argv:
        self_test()
    else:
        gui()
