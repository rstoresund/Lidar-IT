"""Real pye57 + DuckDB integration on generated fixtures, not FARO validation."""
import io
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
import time
import tkinter as tk
import numpy as np
import pye57
from pye57 import libe57
from PIL import Image
from database_backend import DuckDBPointDatabase, ScanSource
from pointcloud_filter import Options, import_to_database, export_from_database, E57Scan
from station_model import inspect_station, direction, suggest_neighbors, project_hotspot, pose
from panorama import ensure_panorama, SphericalRaster, perspective
from scan_viewer import ScanBrowser


def fixture(path):
    with pye57.E57(str(path),'w') as e:
        for i in range(3):
            data={'cartesianX':np.array([1.,1.1,2.,-1.]),
                  'cartesianY':np.array([.1,.1,0.,0.]),
                  'cartesianZ':np.array([2.,1.,0.,0.]),
                  'colorRed':np.array([255,0,0,255],dtype=np.uint8),
                  'colorGreen':np.array([0,255,0,255],dtype=np.uint8),
                  'colorBlue':np.array([0,0,255,255],dtype=np.uint8)}
            e.write_scan_raw(data,name=f'Station {i+1}',translation=np.array([i*5.,0.,0.]))
        # Associated spherical image, explicitly posed at first scanner location.
        root=e.image_file.root();images=root['images2D']
        image=libe57.StructureNode(e.image_file)
        image.set('associatedData3DGuid',libe57.StringNode(e.image_file,root['data3D'][0]['guid'].value()))
        image.set('pose',libe57.StructureNode(e.image_file))
        t=libe57.StructureNode(e.image_file)
        for k in 'xyz':t.set(k,libe57.FloatNode(e.image_file,0.))
        image['pose'].set('translation',t)
        rep=libe57.StructureNode(e.image_file)
        for k,v in [('imageWidth',128),('imageHeight',64)]:rep.set(k,libe57.IntegerNode(e.image_file,v))
        for k,v in [('pixelWidth',2*math.pi/128),('pixelHeight',math.pi/64)]:rep.set(k,libe57.FloatNode(e.image_file,v))
        stream=io.BytesIO();Image.new('RGB',(128,64),(240,20,30)).save(stream,format='PNG')
        payload=stream.getvalue();blob=libe57.BlobNode(e.image_file,len(payload))
        rep.set('pngImage',blob);image.set('sphericalRepresentation',rep);images.append(image)
        blob.write(payload,0,len(payload))


def wait(window):
    deadline=time.monotonic()+20
    while window.worker and window.worker.is_alive():
        window.update();time.sleep(.01)
        assert time.monotonic()<deadline,'GUI worker timeout'
    window.poll_events();window.update()


with TemporaryDirectory() as folder:
    root=Path(folder);src=root/'src';src.mkdir();scratch=root/'scratch';scratch.mkdir()
    source=src/'generated.e57';fixture(source)
    opt=Options(src,root/'out',scratch,output_format='BOTH',chunk_rows=100,grid_size=1.)
    path=root/'project.duckdb';logs=[]
    result,cancel=import_to_database(opt,path,1,False,logs.append,lambda _:None,Event())
    assert not cancel and all(r[1]=='IMPORTED' for r in result),(result,logs)
    again,_=import_to_database(opt,path,1,False,logs.append,lambda _:None,Event())
    assert all(r[1]=='SKIPPED' for r in again)
    with DuckDBPointDatabase(path,scratch,1) as db:
        stations=db.list_stations();assert len(stations)==3
        assert stations[1]['meta']['position']==[5.,0.,0.]
        assert len(stations[0]['meta']['images'])==1
        points=np.concatenate([x for x,c in db.iter_filtered([stations[1]['id']],None,None,None,2)])
        assert points[:,0].min()==4.
        a,b,c=[s['key'] for s in stations]
        db.set_connection(a,b);db.set_connection(b,c)
        assert db.list_connections()==[(a,b),(b,c)]
        assert suggest_neighbors(stations,a)==[b,c]
        assert direction([0,0,0],[1,0,0])==(0.,0.)
        r=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        assert np.allclose(direction([0,0,0],[0,1,0],r),(0.,0.))
        assert project_hotspot([1,0,0],0,0,90,640,360)==(320.,180.)
        assert project_hotspot([-1,0,0],0,0,90,640,360) is None
        stop=Event()
        image,meta=ensure_panorama(db,stations[0],root/'cache',stop,lambda _:None)
        assert meta['panorama_kind']=='E57 spherical image',meta
        with Image.open(image) as im:assert im.getpixel((0,0))==(240,20,30)
        image,meta=ensure_panorama(db,stations[1],root/'cache',stop,lambda _:None)
        assert meta['panorama_kind'].startswith('Derived')
        before=image.stat().st_mtime_ns
        stations[1]['meta']=meta
        ensure_panorama(db,stations[1],root/'cache',stop,lambda _:None)
        assert image.stat().st_mtime_ns==before
        # Incomplete scans stay hidden and abort cleans attached station metadata.
        pending=db.begin_scan(ScanSource('none','pending',0,'e57','x','E57_SCAN_LOCAL',False,True))
        db.save_station(pending,{'position':None});assert len(db.list_stations())==3
        db.abort_scan(pending)
        db.set_connection(a,b,False);assert (a,b) not in db.list_connections();db.set_connection(a,b)
    out,cancel=export_from_database(opt,path,1,False,logs.append,lambda _:None,Event())
    assert not cancel and all(row[1]=='OK' for row in out),(out,logs)
    assert len(list(opt.output_dir.glob('*.las')))==3
    assert len(list(opt.output_dir.glob('*_filtered.csv')))==3
    # Generated-file chunk reader and pose-off metadata remain independently valid.
    opt.apply_e57_pose=False
    scan=E57Scan(source,opt,1);assert not scan.apply_pose
    assert scan.station_metadata['position']==[5.,0.,0.];scan.close()
    raster=SphericalRaster(8,4)
    raster.add(np.array([[2.,0.,0.],[1.,0.,0.]]),np.array([255,65280],np.uint32))
    raster.add(np.array([[3.,0.,0.]]),np.array([16711680],np.uint32))
    assert tuple(np.asarray(raster.image())[2,4])==(0,255,0)
    # GUI initializes and station selection updates map/list/bubble together.
    from unittest.mock import patch
    import customtkinter as ctk
    from pointcloud_filter import gui
    def initialize_app(app):
        app.withdraw();app.update_idletasks()
        assert app.v['cache_dir'].get() is not None
        for job in app.tk.call('after','info'):app.tk.call('after','cancel',job)
        app.destroy()
    with patch.object(ctk.CTk,'mainloop',initialize_app):gui()
    parent=tk.Tk();parent.withdraw();window=ScanBrowser(parent,path,scratch,1)
    wait(window);assert len(window.stations)==3
    window.select_station(a);wait(window)
    assert window.current['key']==a and window.list.curselection()==(0,)
    assert window.panorama is not None
    window.select_station(b);wait(window)
    assert window.current['key']==b and window.list.curselection()==(1,)
    assert window.prev_keys==[a] and window.next_keys==[c]
    window.close_browser();parent.destroy()
    # Early cancellation leaves READY imports and manual links intact.
    stop=Event();stop.set()
    cancelled,cancel=import_to_database(opt,path,1,True,logs.append,lambda _:None,stop)
    assert cancel and not cancelled
    with DuckDBPointDatabase(path,scratch,1) as db:
        assert len(db.list_stations())==3 and (a,b) in db.list_connections()
    # Pose missing is represented explicitly, not silently mapped to (0,0,0).
    class EmptyNode:
        def isDefined(self,name):return False
    assert pose(EmptyNode())==(None,None)
    stop=Event();stop.set()
    try:
        with DuckDBPointDatabase(path,scratch,1) as db:
            ensure_panorama(db,stations[2],root/'cancel_cache',stop,lambda _:None)
        raise AssertionError('Expected panorama cancellation')
    except InterruptedError:pass
    assert not list((root/'cancel_cache').glob('*.png'))
print('PASS: generated native E57 imagery + chunk import, real DuckDB, poses, cache, graph, repeat export, native GUI synchronization')
print('LIMIT: synthetic fixtures only; FARO files and large-dataset benchmarks not validated.')



