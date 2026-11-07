# Changelog

## 3.1.0 — 2026-11-07

- Added persisted E57 station poses, associated imagery metadata, database project identity, and explicit directed navigation relationships.
- Added dark native pan/zoom scan map and mouse-controlled perspective bubble viewer with linked station selection, directional hotspots, upstream/downstream menus, fullscreen, and manual connection editing.
- Added embedded full-spherical PNG/JPEG extraction and cached streamed point RGB/intensity fallback; selectable cache folder.
- Added metadata-only upgrade of existing databases when original unchanged sources remain accessible.
- Fixed native pye57 typed-node chunk-reader compatibility; aligned dependency bounds and declared Pillow.
- Added real DuckDB and generated-native-E57 tests, GUI initialization/synchronization and cancellation checks. Preserved direct filtering and LAS/CSV tests.

## 3.0.0 — supplied baseline

DuckDB import-once/filter-repeatedly workflow and retained SDK-free direct processing.
