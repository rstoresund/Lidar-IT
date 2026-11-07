"""Contract/integration tests using an in-memory fake backend.

Does not require DuckDB or an E57 file. The real DuckDB SQL path must still be
validated on the user's workstation after dependency installation.
Run: python test_database_workflow.py
"""
from __future__ import annotations
import csv
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
from unittest.mock import patch
import numpy as np
from database_backend import ScanRecord
from pointcloud_filter import (Options, ScanJob, import_to_database,
                               export_from_database, scan_source_identity)


class FakeE57:
    def __init__(self,path,opt,scan_index):
        self.rotation=np.eye(3)
        self.translation=np.array([100.,200.,10.])
        self.apply_pose=True
        self.log_hint='Synthetic E57 with scanner origin and applied pose'
    def blocks(self, rows, stop, tick):
        pts=np.array([[0.1,0.2,7.0],[0.2,0.2,1.0],[1.1,0.2,3.0],
                      [2.1,0.1,20.0],[3.5,0.0,np.nan]])
        valid=np.array([True,True,True,True,False])
        tick(1.)
        yield pts,np.array([0x010203,0x040506,0x070809,0x102030,0],dtype=np.uint32),valid
    def close(self): pass


class FakeBackend:
    def __init__(self):
        self.items={}
        self.points={}
        self.counter=0
        self.last_scan_meta=None
    def __enter__(self):return self
    def __exit__(self,*_):pass
    def close(self):pass
    def find_current(self,key):
        all_=[rec for rec in self.items.values() if rec.source_key==key and rec.scan_id in self.points]
        return max(all_,key=lambda r:r.scan_id) if all_ else None
    def list_scans(self):return sorted(self.items.values(),key=lambda r:r.scan_id)
    def begin_scan(self,meta):
        self.counter+=1
        i=self.counter
        self.last_scan_meta=meta
        self.items[i]=ScanRecord(i,meta.source_path,meta.source_key,meta.scan_index,meta.source_format,
                                 meta.source_signature,0,0,meta.coordinate_frame,meta.pose_applied,
                                 meta.scanner_origin_known,'today')
        self.points[i]=[]
        return i
    def append_points(self, scan_id, xyz, rgb, r):
        self.points[scan_id].extend((point.copy(),int(c),float(dist)) for point,c,dist in zip(xyz,rgb,r))
    def complete_scan(self, scan_id, attempted, kept):
        r=self.items[scan_id]
        self.items[scan_id]=ScanRecord(r.scan_id,r.source_path,r.source_key,r.scan_index,r.source_format,r.source_signature,
                                      kept,attempted,r.coordinate_frame,r.pose_applied,r.scanner_origin_known,r.import_utc)
    def abort_scan(self,scan_id):
        self.items.pop(scan_id,None);self.points.pop(scan_id,None)
    def iter_filtered(self, ids, cell, low, high, batch_size):
        allpts=[(xyz,rgb,r) for i in ids for xyz,rgb,r in self.points[i]]
        pts=[(xyz,c,r) for xyz,c,r in allpts if (low is None or r>=low) and (high is None or r<=high)]
        if cell is not None:
            minima={}
            for xyz,c,r in pts:
                key=(int(np.floor(xyz[0]/cell)),int(np.floor(xyz[1]/cell)))
                if key not in minima or xyz[2]<minima[key][0][2]: minima[key]=(xyz,c,r)
            pts=list(minima.values())
        for i in range(0,len(pts),batch_size):
            group=pts[i:i+batch_size]
            yield np.asarray([pt[0] for pt in group]),np.asarray([pt[1] for pt in group],dtype=np.uint32)


with TemporaryDirectory() as temp:
    root=Path(temp)
    src=root/'src';src.mkdir()
    out=root/'out';out.mkdir()
    tmp=root/'scratch';tmp.mkdir()
    fls=src/'scan_01.e57';fls.write_bytes(b'placeholder fixture')
    opt=Options(src,out,tmp,output_format='BOTH',grid_enabled=True,grid_size=1.0,
                range_enabled=True,range_min=0.,range_max=6.,apply_e57_pose=True,chunk_rows=100)
    dbfile=root/'project.duckdb'
    db=FakeBackend()
    msgs=[]
    with patch('database_backend.open_database',return_value=db),patch('pointcloud_filter.job_list',return_value=[ScanJob(fls,0)]),patch('pointcloud_filter.E57Scan',FakeE57):
        r,cancel=import_to_database(opt,dbfile,4,False,msgs.append,lambda _:None,Event())
        assert r[0][1]=='IMPORTED' and not cancel,r
        assert r[0][2]==4, r
        assert db.last_scan_meta.coordinate_frame=='E57_POSED'
        assert db.last_scan_meta.scanner_origin_known
        assert db.items[1].point_count==4
        assert all(pt[0][0]>100 for pt in db.points[1])
        same,_=import_to_database(opt,dbfile,4,False,msgs.append,lambda _:None,Event())
        assert same[0][1]=='SKIPPED',same
    dbfile.write_bytes(b'fake db file for existence test')
    with patch('database_backend.open_database',return_value=db):
        result,cancel=export_from_database(opt,dbfile,4,False,msgs.append,lambda _:None,Event())
    assert not cancel and result[0][1]=='OK',result
    assert result[0][3]==2,result  # lowest in first and second cells; 20m point excluded
    with (out/'scan_01_scan_001_filtered.csv').open(newline='') as f:
        rows=list(csv.DictReader(f))
    assert len(rows)==2,rows
    assert sorted(float(r['z']) for r in rows)==[11.,13.]
    assert (out/'scan_01_scan_001_filtered.las').read_bytes()[:4]==b'LASF'
    assert scan_source_identity(ScanJob(fls,0),opt)[0].endswith('::0')
    # Ensure the contract is not tied to SQL engine specifics in the pipeline.
    assert not list(tmp.glob('pc_grid_*'))

print('PASS: database-interface import, source change detection/skip, pose, range, grid, LAS/CSV export')
print('NOTICE: real DuckDB/E57 execution still requires Windows integration testing.')
