# Implementation notes — 3.1.0

## Codebase assessment and sequence

The supplied v3.0 archive contains an existing CustomTkinter GUI and reader/direct-filter/export pipeline in pointcloud_filter.py; backend-neutral contracts and DuckDB in database_backend.py; standard-library dependency bootstrap; a Windows launcher; and synthetic tests. Dependencies were NumPy, pye57, laspy/lazrs, CustomTkinter, and DuckDB. The direct path uses disk-backed SQLite for minimum-Z cells. The persistent path uses staged DuckDB imports and streamed query/export batches. No viewer or image inspection existed, and scanner transforms were not retained for browsing.

Implementation order: inspect existing readers/contracts and tests; exercise installed native APIs; add independent station geometry and metadata persistence; add bounded panorama caching; integrate synchronized map/bubble navigation; validate native E57 + DuckDB on generated fixtures; preserve and rerun legacy tests; package the complete project.

## Verified findings and fixes

- Native pye57 returns a typed CompressedVectorNode in the installed environment. The v3.0 reader attempted to wrap it again, which failed on generated E57 scans. The reader now accepts typed nodes and converts only generic nodes. The same rule is used for image blobs.
- ScanHeader.has_pose(), low-level buffered point reads, image GUID associations, and BlobNode chunk reads were inspected against the installed API and exercised using generated files.
- Quaternion components are read by name, normalized, and converted to a rigid matrix. Missing or non-finite poses remain explicitly unknown. Poses are preserved independently from the point-coordinate import setting.
- Requirements and bootstrap had conflicting pye57 upper bounds; these now agree. Pillow was added explicitly.
- Cancellation is checked immediately before marking an import READY.
- Existing `_options` collision fix remains intact; the GUI naming regression passes.

## Implementation choices

station_model.py owns E57 station inspection, explicit pose validity, camera projection, and nearest-neighbor suggestions. panorama.py owns fixed-resolution nearest-depth rasterization, extraction and disk cache. scan_viewer.py owns the native Tk window, map, perspective bubble, manual graph controls, and serialized worker tasks. database_backend.py retains the storage boundary with station/connection methods.

Browser alternative evaluated: Pannellum supports equirectangular scenes, yaw/pitch hotspots, tours, and fullscreen (https://pannellum.org/documentation/reference/). This iteration uses Tk + Pillow + NumPy to avoid a WebView runtime, remote CDN, loopback server, and additional frontend build. A future browser renderer can use the same station graph and cache.

Projection follows E57 conventions: Ximage = width/2 - azimuth/pixelWidth; Yimage = height/2 - elevation/pixelHeight. References: https://e57-3d-imgfmt.sourceforge.net/bestCoordinates.html and https://asmaloney.github.io/libE57Format-docs/d9/d2d/structe57_1_1_spherical_representation.html . The native camera and hotspot projection share the same basis, including roll/tilt from inverse station/image rotation.

## Validation and limits

Native runtime tests used Windows Python 3.12, DuckDB 1.5.6, NumPy 2.3.4, Pillow 12.0.0, CustomTkinter 5.2.2, and the locally installed pye57. Tests use generated E57 files, not actual FARO exports. They do not establish production interoperability or large-data throughput.

Point-derived panorama memory is bounded by a 2048×1024 RGB/depth raster plus a 50,000-point batch, independent of scan point count. Perspective display projection is capped at 800×600 before display scaling. DuckDB's memory budget does not cap the whole process. Embedded JPEG/PNG decoding uses Pillow and may allocate the full source image before resizing; Pillow's large-image safeguards remain enabled. Cache PNG writes are atomic; incomplete extraction files are cleaned. No cache quota, image stitching, partial-image reprojection, or gap filling is implemented.

Progress is reported by phase; native image decoding and a DuckDB query may delay cancellation until the next checkpoint. Only one viewer job runs at once; selection during a load requests that the user wait or cancel. Map nearest suggestions are project-distance candidates only. Connections are explicit and persistent, never auto-labeled by proximity. All imported files must already share the intended project frame; no automatic registration or cross-file CRS reconciliation is attempted.

Next priorities: validate a representative FARO E57 with and without images against SCENE coordinates/directions; add fixture coverage for exporter-specific optional metadata and spherical point fields; benchmark large scans; extend partial/pinhole image handling; add cache quota/cleanup; add validated CRS transformation before basemap integration. PostgreSQL remains unimplemented behind the existing adapter contract.
