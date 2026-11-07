"""Station metadata and geometry, independent of storage and GUI."""
from __future__ import annotations
import math
import numpy as np


def value(node, name, default=None):
    return node[name].value() if node.isDefined(name) else default


def pose(node):
    """Return explicit finite rigid pose, never invent an absent station origin."""
    if not node.isDefined('pose/translation'):
        return None, None
    p = node['pose']
    t = np.array([value(p['translation'], k, 0.) for k in 'xyz'], dtype=float)
    r = np.eye(3)
    if p.isDefined('rotation'):
        q = np.array([value(p['rotation'], k, 0.) for k in ('w','x','y','z')], dtype=float)
        norm = np.linalg.norm(q)
        if not np.isfinite(norm) or norm < 1e-12:
            return None, None
        w,x,y,z = q / norm
        r = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                      [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                      [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    if not np.isfinite(t).all() or not np.isfinite(r).all():
        return None, None
    return t.tolist(), r.tolist()


def inspect_station(e57, index):
    root = e57.image_file.root()
    scan = root['data3D'][index]
    position, rotation = pose(scan)
    guid = value(scan, 'guid', '')
    images = []
    if root.isDefined('images2D'):
        for i, image in enumerate(root['images2D']):
            if not guid or value(image, 'associatedData3DGuid') != guid:
                continue
            for kind in ('sphericalRepresentation','cylindricalRepresentation',
                         'pinholeRepresentation','visualReferenceRepresentation'):
                if not image.isDefined(kind):
                    continue
                rep = image[kind]
                ip, ir = pose(image)
                images.append(dict(index=i, representation=kind,
                    width=value(rep,'imageWidth'), height=value(rep,'imageHeight'),
                    pixel_width=value(rep,'pixelWidth'), pixel_height=value(rep,'pixelHeight'),
                    blob='jpegImage' if rep.isDefined('jpegImage') else
                         'pngImage' if rep.isDefined('pngImage') else None,
                    position=ip, rotation=ir))
    return dict(name=value(scan,'name',f'Scan {index+1}'), guid=guid,
                position=position, rotation=rotation, images=images,
                crs=value(root,'coordinateMetadata'), version=1)


def direction(source, target, rotation=None):
    """Project direction into scanner/image frame; +yaw is toward local +Y."""
    d = np.asarray(target, float) - np.asarray(source, float)
    if rotation is not None:
        d = d @ np.asarray(rotation, float)
    if np.linalg.norm(d) < 1e-9:
        return None
    return math.atan2(d[1],d[0]), math.atan2(d[2],np.hypot(d[0],d[1]))


def camera_basis(yaw, pitch):
    f = np.array([math.cos(pitch)*math.cos(yaw),math.cos(pitch)*math.sin(yaw),math.sin(pitch)])
    right = np.array([math.sin(yaw),-math.cos(yaw),0.])
    up = np.cross(right, f)
    return f,right,up


def project_hotspot(vector, yaw, pitch, fov, width, height):
    f,right,up = camera_basis(yaw,pitch)
    v = np.asarray(vector,float)
    depth = v @ f
    if depth <= 0:
        return None
    scale = width / (2*math.tan(math.radians(fov)/2))
    x,y = width/2 + (v@right)*scale/depth, height/2 - (v@up)*scale/depth
    return (x,y) if 0 <= x < width and 0 <= y < height else None


def suggest_neighbors(stations, key, count=2):
    current = next(s for s in stations if s['key']==key)
    if current['meta'].get('position') is None:
        return []
    p = np.asarray(current['meta']['position'])
    # Geometric suggestions only, never inferred upstream/downstream edges.
    return [s['key'] for s in sorted(
        (s for s in stations if s['key']!=key and s['meta'].get('position') is not None),
        key=lambda s: np.linalg.norm(np.asarray(s['meta']['position'])-p))[:count]]
