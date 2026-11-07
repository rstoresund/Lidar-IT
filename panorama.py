"""Bounded-resolution panorama cache; E57 spherical imagery or streamed DB RGB."""
from pathlib import Path
import math
import os
import numpy as np
from PIL import Image
from station_model import camera_basis

WIDTH = 2048
HEIGHT = 1024
CACHE_VERSION = 1


class SphericalRaster:
    def __init__(self, width=WIDTH, height=HEIGHT):
        self.width,self.height = width,height
        self.depth = np.full(width*height,np.inf)
        self.rgb = np.zeros((width*height,3),dtype=np.uint8)

    def add(self, xyz, colors):
        distance = np.linalg.norm(xyz,axis=1)
        valid = np.isfinite(xyz).all(axis=1) & (distance>1e-9)
        xyz,distance,colors = xyz[valid],distance[valid],colors[valid]
        az = np.arctan2(xyz[:,1],xyz[:,0])
        el = np.arctan2(xyz[:,2],np.hypot(xyz[:,0],xyz[:,1]))
        x = (np.floor((.5-az/(2*math.pi))*self.width).astype(int)) % self.width
        y = np.clip(np.floor((.5-el/math.pi)*self.height).astype(int),0,self.height-1)
        pix = y*self.width+x
        order = np.lexsort((distance,pix))
        sorted_pix = pix[order]
        first = np.r_[True, sorted_pix[1:]!=sorted_pix[:-1]] if len(order) else np.array([],bool)
        keep = order[first]
        keep = keep[distance[keep] < self.depth[pix[keep]]]
        self.depth[pix[keep]] = distance[keep]
        c = colors[keep].astype(np.uint32)
        self.rgb[pix[keep]] = np.column_stack((c&255,(c>>8)&255,(c>>16)&255))

    def image(self):
        return Image.fromarray(self.rgb.reshape(self.height,self.width,3))


def perspective(panorama, yaw, pitch, fov=85, width=640, height=360):
    f,right,up = camera_basis(yaw,pitch)
    scale = 2*math.tan(math.radians(fov)/2)/width
    x = (np.arange(width)+.5-width/2)*scale
    y = (height/2-np.arange(height)-.5)*scale
    rays = f + x[None,:,None]*right + y[:,None,None]*up
    az = np.arctan2(rays[:,:,1],rays[:,:,0])
    el = np.arctan2(rays[:,:,2],np.hypot(rays[:,:,0],rays[:,:,1]))
    h,w = panorama.shape[:2]
    u = np.floor((.5-az/(2*math.pi))*w).astype(int)%w
    v = np.clip(np.floor((.5-el/math.pi)*h).astype(int),0,h-1)
    return Image.fromarray(panorama[v,u])


def ensure_panorama(db, station, cache_dir, stop, progress):
    cache_dir = Path(cache_dir)
    cache_dir.mkdir(parents=True,exist_ok=True)
    path = cache_dir / f'scan_{station["id"]}_v{CACHE_VERSION}.png'
    metadata = dict(station['meta'])
    if station['index'] < 0 and metadata.get('position') is None:
        raise ValueError('LAS bubble requires a known scanner origin; reimport with scanner XYZ.')
    if path.is_file() and metadata.get('cache') == str(path):
        return path,metadata
    if stop.is_set():
        raise InterruptedError('Cancelled')
    # Only actual full spherical representations with an explicit camera pose
    # at the scanner centre can safely supply a scanner-location bubble.
    image_error = None
    for image in metadata.get('images',[]):
        w,h = image.get('width'),image.get('height')
        pw,ph = image.get('pixel_width'),image.get('pixel_height')
        if (image['representation']!='sphericalRepresentation' or not image['blob'] or
            not w or not h or not pw or not ph or
            not np.isclose(w*pw,2*math.pi,atol=.02) or not np.isclose(h*ph,math.pi,atol=.02) or
            image['position'] is None or metadata.get('position') is None or
            not np.allclose(image['position'],metadata['position'],atol=.01)):
            continue
        try:
            import pye57
            from pye57 import libe57
            from pointcloud_filter import ScanJob, Options, scan_source_identity
            source = Path(station['path'])
            # Signature includes source stat and import coordinate settings.
            opt = Options(source.parent,source.parent,cache_dir,
                          apply_e57_pose=station['posed'])
            if scan_source_identity(ScanJob(source,station['index']),opt)[1] != station['signature']:
                raise ValueError('E57 source has changed; update database before extracting imagery')
            part = path.with_suffix('.image.partial')
            try:
                with pye57.E57(str(source)) as e:
                    rep = e.image_file.root()['images2D'][image['index']][image['representation']]
                    blob = rep[image['blob']]
                    if not isinstance(blob,libe57.BlobNode):
                        blob = libe57.BlobNode(blob)
                    remaining = blob.byteCount()
                    offset = 0
                    with part.open('wb') as output:
                        while remaining:
                            if stop.is_set(): raise InterruptedError('Cancelled')
                            n = min(1024*1024,remaining)
                            buf = bytearray(n)
                            blob.read(buf,offset,n)
                            output.write(buf)
                            offset += n; remaining -= n
                with Image.open(part) as original:
                    if original.size != (w,h): raise ValueError('E57 image dimensions mismatch')
                    original.thumbnail((WIDTH,HEIGHT))
                    result = original.convert('RGB')
                metadata['view_rotation'] = image['rotation']
                metadata['panorama_kind'] = 'E57 spherical image'
                break
            finally:
                part.unlink(missing_ok=True)
        except InterruptedError:
            raise
        except Exception as exc:
            image_error = str(exc)
    else:
        raster = SphericalRaster()
        progress('Rendering cached RGB/intensity panorama from database...')
        for xyz,colors in db.iter_filtered([station['id']],None,None,None,50000):
            if stop.is_set(): raise InterruptedError('Cancelled')
            if station['index'] < 0:
                xyz = xyz-np.asarray(metadata['position'])
            elif station['posed']:
                xyz = (xyz-np.asarray(metadata['position'])) @ np.asarray(metadata['rotation'])
            raster.add(xyz,colors)
        result = raster.image()
        metadata['view_rotation'] = metadata.get('rotation')
        metadata['panorama_kind'] = 'Derived point RGB/intensity (black = no samples)'
    if stop.is_set(): raise InterruptedError('Cancelled')
    part = path.with_suffix('.partial.png')
    try:
        result.save(part)
        os.replace(part,path)
    finally:
        part.unlink(missing_ok=True)
    metadata.update(cache=str(path), image_error=image_error)
    db.save_station(station['id'],metadata)
    return path,metadata
