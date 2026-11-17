"""Downsample organized XYZ float32 clouds without losing row padding/endian layout."""
import numpy as np


def sample_cloud(data, width, height, point_step, row_step, fields, bigendian=False, stride=4):
    if stride < 1 or width < 1 or height < 1 or point_step < 12 or row_step < width*point_step:
        raise ValueError('Invalid organized cloud layout')
    if len(data) < row_step*height:
        raise ValueError('Truncated cloud data')
    mapping = {f.name: f for f in fields}
    if len(mapping) != len(fields):
        raise ValueError('Duplicate cloud field names')
    for name in ('x','y','z'):
        if name not in mapping or mapping[name].datatype != 7 or mapping[name].count != 1:
            raise ValueError('Expected float32 XYZ fields')
        if not 0 <= mapping[name].offset <= point_step-4:
            raise ValueError('XYZ field outside point record')
    records = np.ndarray((height,width),dtype=np.dtype('V'+str(point_step)),
                         buffer=data,strides=(row_step,point_step))[::stride,::stride].copy().ravel()
    dtype = np.dtype(dict(names=['x','y','z'],formats=[('>' if bigendian else '<')+'f4']*3,
                          offsets=[mapping[k].offset for k in ('x','y','z')],itemsize=point_step))
    xyz = records.view(dtype)
    valid = np.isfinite(xyz['x']) & np.isfinite(xyz['y']) & np.isfinite(xyz['z'])
    return records[valid].tobytes()
