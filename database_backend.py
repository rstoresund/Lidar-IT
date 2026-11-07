"""PointCloud Filter v3 - backend-neutral point repository contract and DuckDB.

All point coordinates and ranges in the persistent repository are METRES.
The public API is intentionally independent of DuckDB so that a future
PostgreSQL/PostGIS implementation can implement exactly the same contract.
"""
from __future__ import annotations

import json
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator
import numpy as np


@dataclass(frozen=True)
class ScanRecord:
    scan_id: int
    source_path: str
    source_key: str
    scan_index: int
    source_format: str
    source_signature: str
    point_count: int
    attempted_count: int
    coordinate_frame: str
    pose_applied: bool
    scanner_origin_known: bool
    import_utc: str

    @property
    def label(self) -> str:
        name = Path(self.source_path).name
        return f'{name} [scan {self.scan_index+1}]' if self.scan_index >= 0 else name

    @property
    def output_name(self) -> str:
        name = Path(self.source_path).stem
        return f'{name}_scan_{self.scan_index+1:03d}' if self.scan_index >= 0 else name


@dataclass(frozen=True)
class ScanSource:
    source_path: str
    source_key: str
    scan_index: int
    source_format: str
    source_signature: str
    coordinate_frame: str
    pose_applied: bool
    scanner_origin_known: bool


class PointDatabase(ABC):
    """Database contract: a second engine need only implement these operations.

    A scan is added in stages (begin, append many chunks, complete). A failed
    scan never becomes READY. Filter queries return small NumPy batches.
    """
    @abstractmethod
    def find_current(self, source_key: str) -> ScanRecord | None: ...

    @abstractmethod
    def list_scans(self) -> list[ScanRecord]: ...

    @abstractmethod
    def begin_scan(self, meta: ScanSource) -> int: ...

    @abstractmethod
    def append_points(self, scan_id: int, xyz_m: np.ndarray,
                      rgb: np.ndarray, ranges_m: np.ndarray | None) -> None: ...

    @abstractmethod
    def complete_scan(self, scan_id: int, attempted: int, kept: int) -> None: ...

    @abstractmethod
    def abort_scan(self, scan_id: int) -> None: ...

    @abstractmethod
    def iter_filtered(self, scan_ids: list[int], cell_m: float | None,
                      minimum_m: float | None, maximum_m: float | None,
                      batch_size: int) -> Iterator[tuple[np.ndarray, np.ndarray]]: ...

    @abstractmethod
    def close(self) -> None: ...

    def save_station(self, scan_id: int, metadata: dict) -> None:
        raise NotImplementedError('Station metadata is unsupported by this backend.')

    def list_stations(self) -> list[dict]:
        raise NotImplementedError

    def list_connections(self) -> list[tuple[str, str]]:
        raise NotImplementedError

    def set_connection(self, source: str, target: str, connected: bool = True) -> None:
        raise NotImplementedError

    def __enter__(self): return self
    def __exit__(self, *_): self.close()


class DuckDBPointDatabase(PointDatabase):
    """Single-writer local DB, with DuckDB memory + temp files on chosen disk.

    File-backed persistent storage. No server. A single app/process should open
    the file for writing at once. DB and scratch should preferably be on SSD.
    """

    def __init__(self, path: Path, scratch: Path, memory_gb: int = 8):
        try:
            import duckdb
        except ImportError as exc:
            raise RuntimeError('DuckDB missing. Install: python -m pip install duckdb') from exc
        if not 1 <= int(memory_gb) <= 256:
            raise ValueError('DuckDB memory limit must be between 1 and 256 GiB.')
        self.path = Path(path).expanduser().resolve()
        self.scratch = Path(scratch).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.scratch.mkdir(parents=True, exist_ok=True)
        self.con = duckdb.connect(str(self.path))
        # DuckDB SET statements don't accept parameters on all versions; quote
        # the user-selected path for SQL string-literal safety.
        safe_temp = str(self.scratch / 'duckdb_spill').replace("'", "''")
        (self.scratch / 'duckdb_spill').mkdir(parents=True, exist_ok=True)
        self.con.execute(f"SET temp_directory='{safe_temp}'")
        self.con.execute(f"SET memory_limit='{int(memory_gb)}GB'")
        self._init_schema()

    def _init_schema(self):
        self.con.execute("CREATE SEQUENCE IF NOT EXISTS pc_scan_ids START 1")
        self.con.execute('''
            CREATE TABLE IF NOT EXISTS pc_scans (
                scan_id BIGINT PRIMARY KEY DEFAULT nextval('pc_scan_ids'),
                source_path VARCHAR NOT NULL,
                source_key VARCHAR NOT NULL,
                scan_index INTEGER NOT NULL,
                source_format VARCHAR NOT NULL,
                source_signature VARCHAR NOT NULL,
                coordinate_frame VARCHAR NOT NULL,
                pose_applied BOOLEAN NOT NULL,
                scanner_origin_known BOOLEAN NOT NULL,
                status VARCHAR NOT NULL,
                attempted_count BIGINT DEFAULT 0,
                point_count BIGINT DEFAULT 0,
                import_utc TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        self.con.execute('''
            CREATE TABLE IF NOT EXISTS pc_points (
                scan_id BIGINT NOT NULL,
                x DOUBLE NOT NULL,
                y DOUBLE NOT NULL,
                z DOUBLE NOT NULL,
                rgb UINTEGER NOT NULL,
                range_m DOUBLE
            )
        ''')
        # Columnar clustering by scan_id during INSERT already helps zone-map
        # pruning. A secondary index for every point would inflate the DB.

        self.con.execute('CREATE TABLE IF NOT EXISTS pc_project (project_id VARCHAR PRIMARY KEY, name VARCHAR)')
        self.con.execute("INSERT INTO pc_project SELECT CAST(uuid() AS VARCHAR), ? WHERE NOT EXISTS (SELECT 1 FROM pc_project)", [self.path.stem])
        self.con.execute('CREATE TABLE IF NOT EXISTS pc_stations (scan_id BIGINT PRIMARY KEY, metadata VARCHAR NOT NULL)')
        self.con.execute('CREATE TABLE IF NOT EXISTS pc_connections (source_key VARCHAR, target_key VARCHAR, PRIMARY KEY(source_key,target_key))')

    def save_station(self, scan_id, metadata):
        self.con.execute('INSERT OR REPLACE INTO pc_stations VALUES (?,?)',
                         [int(scan_id), json.dumps(metadata, allow_nan=False)])

    def list_stations(self):
        records = self.list_scans()
        metadata = {int(i): json.loads(m) for i,m in self.con.execute(
            'SELECT s.scan_id,s.metadata FROM pc_stations s JOIN pc_scans c USING(scan_id) WHERE c.status=\'READY\'').fetchall()}
        return [dict(id=r.scan_id, key=r.source_key, label=r.label, path=r.source_path,
                     index=r.scan_index, signature=r.source_signature, posed=r.pose_applied,
                     meta=metadata.get(r.scan_id,dict(name=r.label,position=None,rotation=None,images=[])))
                for r in records]

    def list_connections(self):
        keys = {s.source_key for s in self.list_scans()}
        return [(a,b) for a,b in self.con.execute('SELECT source_key,target_key FROM pc_connections ORDER BY source_key,target_key').fetchall()
                if a in keys and b in keys]

    def set_connection(self, source, target, connected=True):
        keys = {s.source_key for s in self.list_scans()}
        if source == target or source not in keys or target not in keys:
            raise ValueError('Connections require two distinct current stations.')
        if connected:
            self.con.execute('INSERT OR IGNORE INTO pc_connections VALUES (?,?)',[source,target])
        else:
            self.con.execute('DELETE FROM pc_connections WHERE source_key=? AND target_key=?',[source,target])

    @staticmethod
    def _from_row(row) -> ScanRecord:
        return ScanRecord(int(row[0]), str(row[1]), str(row[2]), int(row[3]),
                          str(row[4]), str(row[5]), int(row[6]), int(row[7]),
                          str(row[8]), bool(row[9]), bool(row[10]), str(row[11]))

    _select = '''SELECT scan_id, source_path, source_key, scan_index, source_format,
                       source_signature, point_count, attempted_count,
                       coordinate_frame, pose_applied, scanner_origin_known,
                       CAST(import_utc AS VARCHAR) FROM pc_scans'''

    def find_current(self, source_key: str) -> ScanRecord | None:
        row = self.con.execute(self._select + '''
            WHERE status = 'READY' AND source_key = ?
            ORDER BY scan_id DESC LIMIT 1''', [source_key]).fetchone()
        return self._from_row(row) if row else None

    def list_scans(self) -> list[ScanRecord]:
        # Return the latest COMPLETE scan for each source key. Staged or failed
        # scans are never accidentally included in filtering queries.
        rows = self.con.execute(self._select + '''
            WHERE status = 'READY'
              AND scan_id = (SELECT MAX(b.scan_id) FROM pc_scans b
                             WHERE b.source_key = pc_scans.source_key AND b.status = 'READY')
            ORDER BY source_path, scan_index''').fetchall()
        return [self._from_row(row) for row in rows]

    def begin_scan(self, meta: ScanSource) -> int:
        row = self.con.execute('''
            INSERT INTO pc_scans
                (source_path, source_key, scan_index, source_format,
                 source_signature, coordinate_frame, pose_applied,
                 scanner_origin_known, status)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, 'IMPORTING') RETURNING scan_id''',
            [meta.source_path, meta.source_key, meta.scan_index, meta.source_format,
             meta.source_signature, meta.coordinate_frame, meta.pose_applied,
             meta.scanner_origin_known]).fetchone()
        return int(row[0])

    def append_points(self, scan_id: int, xyz_m: np.ndarray,
                      rgb: np.ndarray, ranges_m: np.ndarray | None) -> None:
        if not len(xyz_m): return
        if xyz_m.ndim != 2 or xyz_m.shape[1] != 3:
            raise ValueError('XYZ chunk must have shape Nx3.')
        if len(rgb) != len(xyz_m) or (ranges_m is not None and len(ranges_m) != len(xyz_m)):
            raise ValueError('XYZ/RGB/range arrays must match in length.')
        data = {'x': np.ascontiguousarray(xyz_m[:,0],dtype=np.float64),
                'y': np.ascontiguousarray(xyz_m[:,1],dtype=np.float64),
                'z': np.ascontiguousarray(xyz_m[:,2],dtype=np.float64),
                'rgb': np.ascontiguousarray(rgb,dtype=np.uint32),
                'range_m': np.ascontiguousarray(ranges_m,dtype=np.float64) if ranges_m is not None
                           else np.full(len(rgb),np.nan,dtype=np.float64)}
        # NumPy column dictionaries are supported directly by DuckDB's Python
        # replacement scans/register API. Do not execute millions of INSERTs.
        # NULL range (not nan) deliberately means no scanner origin for LAS.
        self.con.register('__pc_import_batch', data)
        try:
            if ranges_m is None:
                self.con.execute('''
                    INSERT INTO pc_points SELECT ?, x, y, z, rgb, NULL
                    FROM __pc_import_batch''', [int(scan_id)])
            else:
                self.con.execute('''
                    INSERT INTO pc_points SELECT ?, x, y, z, rgb, range_m
                    FROM __pc_import_batch''', [int(scan_id)])
        finally:
            self.con.unregister('__pc_import_batch')

    def complete_scan(self, scan_id: int, attempted: int, kept: int) -> None:
        self.con.execute('''UPDATE pc_scans SET status='READY', attempted_count=?,
                        point_count=? WHERE scan_id=? AND status='IMPORTING' ''',
                        [int(attempted), int(kept), int(scan_id)])
        # Historical versions are retained until explicit cleanup, protecting
        # the database against a failed replacement import. list_scans ignores
        # them; this is documented as a possible storage cost.

    def abort_scan(self, scan_id: int) -> None:
        # Best effort cleanup: if disk is full, failed imports may be left as
        # IMPORTING (never eligible for filtering). Reimport is still safe.
        try:
            self.con.execute('DELETE FROM pc_stations WHERE scan_id=?', [int(scan_id)])
            self.con.execute('DELETE FROM pc_points WHERE scan_id=?', [int(scan_id)])
            self.con.execute('DELETE FROM pc_scans WHERE scan_id=?', [int(scan_id)])
        except Exception:
            try:
                self.con.execute("UPDATE pc_scans SET status='FAILED' WHERE scan_id=?", [int(scan_id)])
            except Exception: pass

    def iter_filtered(self, scan_ids: list[int], cell_m: float | None,
                      minimum_m: float | None, maximum_m: float | None,
                      batch_size: int) -> Iterator[tuple[np.ndarray, np.ndarray]]:
        if not scan_ids:
            raise ValueError('No database scans selected.')
        if cell_m is not None and (not math.isfinite(cell_m) or cell_m <= 0):
            raise ValueError('Grid size must be a positive finite number.')
        if minimum_m is not None and (not math.isfinite(minimum_m) or minimum_m < 0):
            raise ValueError('Minimum range must be nonnegative.')
        if maximum_m is not None and (not math.isfinite(maximum_m) or
                                      (minimum_m is not None and maximum_m < minimum_m)):
            raise ValueError('Invalid maximum range.')
        if minimum_m is not None or maximum_m is not None:
            count = int(self.con.execute('''
                SELECT COUNT(*) FROM pc_scans WHERE scan_id IN (''' + ','.join('?'*len(scan_ids)) +
                ''') AND scanner_origin_known = FALSE''', scan_ids).fetchone()[0])
            if count:
                raise ValueError('Scanner-range filtering is unavailable for at least one imported LAS scan. '
                                 'Reimport it with scanner XYZ origin, or disable range filtering.')
        placeholders=','.join('?' for _ in scan_ids)
        where=[f'scan_id IN ({placeholders})']
        params=list(map(int,scan_ids))
        if minimum_m is not None:
            where.append('range_m >= ?')
            params.append(float(minimum_m))
        if maximum_m is not None:
            where.append('range_m <= ?')
            params.append(float(maximum_m))
        where_sql=' AND '.join(where)
        if cell_m is None:
            sql=f'SELECT x, y, z, rgb FROM pc_points WHERE {where_sql}'
        else:
            # arg_min returns the WHOLE original lowest-Z point (not a sampled
            # cell-center coordinate). FLOOR handles negative X/Y correctly.
            sql=f'''SELECT minimum_point.x, minimum_point.y, minimum_point.z,
                           minimum_point.rgb
                    FROM (
                      SELECT ARG_MIN(STRUCT_PACK(x := x, y := y, z := z, rgb := rgb), z)
                             AS minimum_point
                      FROM pc_points WHERE {where_sql}
                      GROUP BY FLOOR(x / ?), FLOOR(y / ?)
                    ) cells'''
            params += [float(cell_m), float(cell_m)]
        # DuckDB's aggregation can spill on disk under its memory limit. Fetch
        # final results in bounded batches rather than fetchall/fetchnumpy.
        cursor=self.con.execute(sql, params)
        while True:
            rows=cursor.fetchmany(batch_size)
            if not rows: break
            arr=np.asarray(rows,dtype=np.float64)
            yield np.asarray(arr[:,:3],dtype=np.float64), np.asarray(arr[:,3],dtype=np.uint32)

    def close(self) -> None:
        con=getattr(self,'con',None)
        if con is not None:
            self.con=None
            con.close()


class PostgreSQLPointDatabase(PointDatabase):
    """Reserved extension point. PostgreSQL is NOT implemented in v3.0.

    An adapter will need the same method signatures and metre-based storage,
    while using COPY for batch insertion and server-side cursors for export.
    """
    def __init__(self, *_, **__):
        raise NotImplementedError('PostgreSQL backend is not implemented; use DuckDB in v3.0.')
    def find_current(self, source_key): raise NotImplementedError
    def list_scans(self): raise NotImplementedError
    def begin_scan(self, meta): raise NotImplementedError
    def append_points(self, scan_id, xyz_m, rgb, ranges_m): raise NotImplementedError
    def complete_scan(self, scan_id, attempted, kept): raise NotImplementedError
    def abort_scan(self, scan_id): raise NotImplementedError
    def iter_filtered(self, scan_ids, cell_m, minimum_m, maximum_m, batch_size): raise NotImplementedError
    def close(self): pass


def open_database(engine: str, path: Path, scratch: Path, memory_gb: int = 8) -> PointDatabase:
    if engine == 'DuckDB':
        return DuckDBPointDatabase(path, scratch, memory_gb)
    if engine == 'PostgreSQL':
        return PostgreSQLPointDatabase()
    raise ValueError(f'Unknown database engine: {engine}')
