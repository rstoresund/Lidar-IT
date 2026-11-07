"""Run: python test_pipeline.py
Synthetic end-to-end tests. E57 decoding is not validated without a real E57 sample.
"""
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
from unittest.mock import patch
import csv
import numpy as np
from pointcloud_filter import Options, ScanJob, process_one, LAS_POINT2_DTYPE

class FakeE57Scan:
    def __init__(self, path, opt, scan_index):
        assert scan_index==0
        self.rotation=np.eye(3)
        self.translation=np.array([100.0,200.0,50.0])
        self.apply_pose=True
        self.output_scale=1.0
        self.log_hint='Synthetic E57'
    def blocks(self, rows, cancel, tick):
        points=np.array([
            [2.1,0.1,4.0], # cell 102,200 -> replaced
            [2.2,0.1,1.5], # cell 102,200 -> kept
            [1.1,1.1,3.0], # cell 101,201 -> kept
            [0.2,0.2,6.0], # outside range
            [8.0,0.2,2.0], # outside range
            [0.0,0.0,0.0], # invalid
        ])
        colors=np.array([0x111111,0x222222,0x333333,0x444444,0x555555,0],dtype=np.uint32)
        valid=np.array([True,True,True,True,True,False])
        tick(1.0)
        yield points, colors, valid
    def close(self): pass

with TemporaryDirectory() as folder:
    root=Path(folder)
    source=root/'source';source.mkdir()
    output=root/'out';output.mkdir()
    scratch=root/'scratch';scratch.mkdir()
    (source/'scan.e57').write_bytes(b'dummy')
    opt=Options(source,output,scratch,output_format='BOTH',grid_enabled=True,grid_size=1.0,
                range_enabled=True,range_min=0.5,range_max=5,apply_e57_pose=True)
    with patch('pointcloud_filter.E57Scan',FakeE57Scan):
        stats,paths=process_one(ScanJob(source/'scan.e57',0),opt,Event(),lambda p:None,lambda s:None)
    assert (stats.attempted,stats.valid,stats.in_range,stats.kept)==(6,5,3,2),stats
    assert len(paths)==2
    with (output/'scan_scan_001_filtered.csv').open(newline='') as f:
        data=list(csv.DictReader(f))
    assert len(data)==2,data
    assert sorted(float(row['z']) for row in data)==[51.5,53.0]
    assert sorted(float(row['x']) for row in data)==[101.1,102.2]
    raw=(output/'scan_scan_001_filtered.las').read_bytes()
    assert raw[:4]==b'LASF' and len(raw)==227+2*LAS_POINT2_DTYPE.itemsize
    assert list(scratch.iterdir())==[],list(scratch.iterdir())
    assert (source/'scan.e57').read_bytes()==b'dummy'
print('PASS: E57-style pipeline, range before pose, grid minimum-Z, LAS/CSV output, scratch cleanup')
