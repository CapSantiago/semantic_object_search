"""Downsample organized XYZ float32 clouds without losing row padding/endian layout."""
import numpy as np


def _sample_records(data, width, height, point_step, row_step, fields, bigendian=False, stride=4):
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
    return records,xyz


def sample_cloud(data, width, height, point_step, row_step, fields, bigendian=False, stride=4, compact=False):
    records,xyz=_sample_records(data,width,height,point_step,row_step,fields,bigendian,stride)
    valid = np.isfinite(xyz['x']) & np.isfinite(xyz['y']) & np.isfinite(xyz['z'])
    if compact:
        return np.column_stack([xyz[k][valid] for k in ('x','y','z')]).astype('<f4').tobytes()
    return records[valid].tobytes()


def clearing_cloud(data,width,height,point_step,row_step,fields,bigendian,k,max_depth,stride=4):
    """Fortress X-forward cloud: only +Inf X is verified no return before clip.

    Ground and finite returns remain real endpoints. NaN/zero/negative Inf
    never become free rays. This output must be used for clearing only.
    """
    _,xyz=_sample_records(data,width,height,point_step,row_step,fields,bigendian,stride)
    k=np.asarray(k,dtype=float)
    if k.shape!=(9,)or not np.isfinite(k).all()or k[0]<=0 or k[4]<=0 or not np.isfinite(max_depth)or max_depth<=0:
        raise ValueError('Invalid clearing camera calibration')
    points=np.column_stack([xyz[name]for name in('x','y','z')])
    valid=np.isfinite(points).all(axis=1)&(points[:,0]>0)&(points[:,0]<=max_depth+.001)
    y,x=np.mgrid[0:height:stride,0:width:stride]
    # Fortress samples pixel centres while its CameraInfo principal point is w/2,h/2.
    rays=np.column_stack([np.ones(x.size),-(x.ravel()+.5-k[2])/k[0],-(y.ravel()+.5-k[5])/k[4]])
    if valid.sum()<8:
        return points[valid].astype('<f4').tobytes(),0
    residual=np.linalg.norm(rays[valid]*points[valid,0,None]-points[valid],axis=1)
    if float(np.quantile(residual,.95))>.002:
        raise ValueError('Cloud and camera intrinsics do not match verified Fortress geometry')
    no_return=np.isposinf(points[:,0])
    output=np.concatenate([points[valid],rays[no_return]*max_depth])
    return output.astype('<f4').tobytes(),int(no_return.sum())
