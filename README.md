# PointCloud Filter 3.1.0 — Scan Map + Bubble Viewer prototype

Windows desktop, Python 3.12, SDK-free E57/LAS/LAZ import with persistent DuckDB points, repeatable range/grid filtering, and LAS/CSV export. The existing v3.0 pipeline is retained.

## Run on Windows

Extract the complete project folder and double-click `Start_PointCloud_Filter.bat`. Alternatively, in PowerShell in the project folder:

```powershell
py -3.12 -m pip install -r requirements.txt
py -3.12 pointcloud_filter.py
```

The launcher uses its own `.venv`; IDLE/F5 uses IDLE's interpreter. The automatic installer uses the launching interpreter. No FARO SDK, browser runtime, or online map service is required.

## Import → map → bubble → adjacent station

1. Select input folder, local SSD DuckDB file, scratch directory, and memory budget. Normally leave Apply E57 scan pose enabled. Import / Update DB preserves original-resolution valid points. Unchanged scans are skipped.
2. Click **Scan Map + Bubble Viewer** after import finishes. It opens a separate dark window. Existing v3 databases gain metadata tables without rebuilding their point tables. Metadata for older E57 imports is recovered only if the original source and import signature still match.
3. Click a station on the map or in the station list. Drag the map to pan and use the wheel to zoom. Positions remain in project/local XY meters. Scans with missing poses stay in the list.
4. Drag the bubble to look around; use the wheel to zoom. Fullscreen toggles the browser window; Escape leaves fullscreen. The status describes the actual panorama source and any image extraction fallback.
5. Select a target in **Selected station → downstream**, then Add connection. This explicitly defines flow order. Reverse the selected/target stations to define the opposite direction. Remove connection deletes that directed relationship. Branching is allowed. Previous/upstream and Next/downstream menus show the corresponding connections.
6. Click a visible navigation marker in the bubble to move to a connected station. The map and list selection follow. **Suggest nearest** offers geometric candidates without automatically inferring upstream/downstream.
7. Close the browser to return to filtering. The modal window prevents parent import/export operations from competing with browsing database operations.

Default panorama cache: a `<database_stem>_panoramas` directory beside the database. The optional **Panorama cache** folder in the main GUI relocates it, e.g. to another SSD. Cache names include immutable import IDs and a rendering version. Delete cached PNGs while the viewer is closed to reclaim space; they regenerate on demand. Old imports and their old cached assets are retained; there is no automatic quota or eviction yet.

## Imagery behavior

The importer inspects `images2D` and associates images by the scan GUID. Full spherical PNG/JPEG imagery is preferred when its angular coverage is approximately 360° × 180° and an explicit finite image pose places its camera within 1 cm of the scanner. Image orientation is used for navigation directions.

Partial spherical, cylindrical, pinhole, visual-reference, unassociated, or unposed imagery is not misrepresented as a full scanner-centered panorama. These scans use the streamed point RGB/intensity fallback. Unsupported associated image representations remain recorded in metadata.

The fallback retains the nearest sample per spherical pixel. Black pixels mean no samples, not inferred scene content. It is a perspective look-around view of a cached 2048 × 1024 panorama, not a reconstructed photographic surface. Local scanner coordinates are recovered from posed database points. Poses are retained even when point-coordinate pose application is disabled. LAS needs an explicitly supplied scanner origin for bubble generation.

## Filtering and storage

Range is measured before E57 pose application. The database retains original valid XYZ, packed RGB/grayscale, and scanner distance in meters. Grid filtering retains the actual minimum-Z point per horizontal XY cell; it is not ground classification. Individual and combined queries and direct-file mode remain available. Combine only scans known to share a coordinate system. Output units are user-selectable. LAS 1.2 output has RGB but no source CRS, raw intensity, or classification metadata.

Imports remain staged until all points and metadata complete. Cancellation discards incomplete imports. A failed replacement leaves the previous completed import available. Older completed point versions remain stored but hidden from current queries. Back up databases and avoid concurrent writers. Do not disconnect storage during operations.

## Tests

```powershell
py -3.12 pointcloud_filter.py --self-test
py -3.12 test_dependencies.py
py -3.12 test_gui_regression.py
py -3.12 test_pipeline.py
py -3.12 test_database_workflow.py
py -3.12 test_scan_viewer.py
```

The new test uses actual pye57/libE57 and DuckDB with programmatically generated E57 scans and an embedded spherical image. It checks chunk import, pose metadata, image extraction, point-derived panoramas, nearest-depth sampling, cache reuse, directed graph editing, repeated export, missing pose handling, cancellation, CustomTkinter initialization, and viewer selection synchronization. GUI tests require a Windows desktop session and briefly create windows.

**Not yet validated:** real FARO-exported E57 files, visual agreement with SCENE, very large image decoding, hundreds of millions of points, workstation RAM/SSD benchmarks, or geographic basemaps. A CRS string is retained when present, but this prototype uses local/project XY only and does not transform CRS coordinates.

See `IMPLEMENTATION_NOTES.md`, `ARCHITECTURE.md`, and `CHANGELOG.md` for the audit and implementation boundary.

## Repository versions

Application source lives at the repository root. To run a downloaded version, extract the repository archive and double-click `Start_PointCloud_Filter.bat`. Keep all source files together.

Stable versions use Git tags such as `v3.1.0`. On GitHub, choose the tag from the branch/tag selector and use **Code → Download ZIP** to download that version. When updating, extract into a new folder and select your existing project database from the app. Keep databases and source scans outside the checkout; they are excluded from Git.
