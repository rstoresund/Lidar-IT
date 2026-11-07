# Storage architecture and contract

`database_backend.py` defines the backend-independent `PointDatabase` contract, and the `DuckDBPointDatabase` implementation. Persistent files are intended to live on a **local SSD**, not to be shared for simultaneous write operations.

## Contract methods

| Method | Behavior |
|---|---|
| `find_current(source_key)` | Fetch latest completed scan for one source scan |
| `list_scans()` | Enumerate only latest READY imports, excluding failed/staged imports |
| `begin_scan(ScanSource)` | Stage metadata and return new scan ID |
| `append_points(scan_id, xyz_m, rgb, range_m)` | Append NumPy-sized chunks into the backend; RGB packed 8-bit |
| `complete_scan(scan_id, attempted, kept)` | Make finished import queryable |
| `abort_scan(scan_id)` | Best-effort delete incomplete import |
| `iter_filtered(ids, grid_m, min_range_m, max_range_m, chunk_size)` | Return filtered point batches in meters |
| `close()` | Release backend connection |

The GUI and all pipeline functions only call these methods. `open_database` selects the backend. Only `DuckDB` is available. `PostgreSQL` is a clear future adapter boundary **not yet implemented**.

## DuckDB schema (tables `pc_scans`, `pc_points`)

`pc_scans` stores one record per import attempt: absolute path, source key, scan index, input format, source signature (path, size, modification timestamp + import settings), coordinate-frame indicator, pose applied flag, known scanner origin flag, imported point count, attempted count, UTC import time, and status.

`pc_points` stores `scan_id`, `x`, `y`, `z` (FLOAT64 meters), `rgb` (packed UINT32), and `range_m` (FLOAT64 or NULL if original LAS scanner position was not supplied).

Only completed most-recent imports are queried. A repeated import of changed source stages new rows first, so an unsuccessful replacement cannot corrupt the previous import. Old completed and crash-staged rows still occupy storage and need future explicit cleanup. This initial schema favors reproducibility over aggressive garbage collection.

## Filtering

The database filters scanner range using `range_m` first. If grid thinning is enabled, `FLOOR(x/cell_m)` and `FLOOR(y/cell_m)` define XY cells, and `ARG_MIN(STRUCT_PACK(...), z)` returns one actual lowest-elevation retained input point per occupied cell. Aggregation may spill to the selected SSD scratch directory under DuckDB's memory limit. If grid thinning is disabled, eligible points stream directly from the DB. Results are fetched in bounded batches and written to the existing incremental LAS/CSV writer. When per-scan mode is chosen, one query per completed scan avoids accidental mixing of coordinate frames.

## PostgreSQL migration path

Add an implementation of the same contract with `psycopg` using bulk `COPY` into Postgres for inserts, a server-side cursor for streamed results, and a SQL aggregate equivalent to DuckDB's `arg_min`. PostGIS / pgPointcloud is optional and would require a separate storage/benchmark evaluation. A PostgreSQL adapter should preserve the exact semantics of `scan_id`, `source_key`, meters, range, and minimum-Z cell alignment.

Before large production use, test one E57 against SCENE reference points, verify transformed XYZ and range, compare the output LAS with QGIS/CloudCompare, and benchmark disk/RAM on the target external SSD.

## 3.1 additive station/navigation schema

`pc_project` stores one UUID/name per database. Existing `pc_scans` records remain the source-file/import version records (path, signature, scan index, status, frame, timestamps). `pc_stations` stores JSON metadata by immutable import scan ID: source scan GUID/name, finite project position and rotation or explicit nulls, image descriptors/poses, optional CRS string, and cache metadata. READY/current scan selection remains defined by the existing backend query. `pc_connections` stores directed source-key → target-key edges, so connections survive new import version IDs. Aborted imports clean station metadata too; old completed imports remain retained as before.

Public additions to PointDatabase: save_station, list_stations, list_connections, set_connection. GUI and panorama code use this interface rather than SQL. New tables are created with IF NOT EXISTS; point tables and point rows are not rebuilt. Existing metadata can be inspected from unchanged source E57 files when opening the viewer.

Worker threads own their database connections. The modal browser prevents the parent GUI from starting competing write operations. Point-derived images stream unfiltered batches through the existing contract; export algorithms remain separate. All station/viewer math is meters/radians; image and station rotations are local-to-project matrices.
