"""Depth-to-XYZ for a Fortress camera, gated by independent native calibration.

The half-pixel convention is verified against that sensor's own native cloud;
it is not a general correction for real RGB-D drivers. No scene data is used.
"""
import numpy as np


def depth_xyz(data,width,height,step,bigendian,encoding,k,stride=4):
    if type(stride)is not int or stride<1 or width<1 or height<1:
        raise ValueError('Invalid depth image dimensions or stride')
    formats={'32FC1':('f4',4,1.),'16UC1':('u2',2,.001)}
    if encoding not in formats:raise ValueError('Unsupported depth encoding')
    fmt,size,scale=formats[encoding]
    if step<width*size or len(data)<step*height:raise ValueError('Truncated depth image')
    k=np.asarray(k,dtype=float)
    if k.shape!=(9,)or not np.isfinite(k).all()or k[0]<=0 or k[4]<=0:
        raise ValueError('Invalid camera intrinsics')
    image=np.ndarray((height,width),dtype=('>'if bigendian else'<')+fmt,
        buffer=data,strides=(step,size))[::stride,::stride]
    z=image.astype(np.float64).ravel()*scale
    y,x=np.mgrid[0:height:stride,0:width:stride]
    rays=np.column_stack([np.ones(x.size),-(x.ravel()+.5-k[2])/k[0],-(y.ravel()+.5-k[5])/k[4]])
    with np.errstate(invalid='ignore'):
        xyz=rays*z[:,None]
    xyz[z<=0]=np.nan
    return xyz,rays


def verify_native_projection(native,projected,tolerance=.002):
    """Independent native coordinates/masks, never a self-projection check."""
    native=np.asarray(native,dtype=float);projected=np.asarray(projected,dtype=float)
    if native.shape!=projected.shape or native.ndim!=2 or native.shape[1]!=3:
        raise ValueError('Native and projected sampling dimensions differ')
    valid=np.isfinite(native).all(axis=1);other=np.isfinite(projected).all(axis=1)
    if not np.array_equal(valid,other):raise ValueError('Native finite-return mask differs')
    if not np.array_equal(np.isposinf(native[:,0]),np.isposinf(projected[:,0])):
        raise ValueError('Native no-return mask differs')
    if valid.sum()<8:raise ValueError('Insufficient finite native calibration points')
    error=np.linalg.norm(native[valid]-projected[valid],axis=1)
    if float(error.max())>tolerance:raise ValueError('Native and projected coordinates differ')
    return dict(finite_points=int(valid.sum()),p95_error_m=float(np.quantile(error,.95)),
                maximum_error_m=float(error.max()))


def projected_clouds(xyz,rays,max_depth):
    """Actual finite marking points; +Inf endpoints belong to clearing only."""
    xyz=np.asarray(xyz);rays=np.asarray(rays)
    if xyz.shape!=rays.shape or xyz.ndim!=2 or xyz.shape[1]!=3:
        raise ValueError('Depth coordinates and rays differ')
    if not np.isfinite(max_depth)or max_depth<0:raise ValueError('Invalid depth far clip')
    finite=np.isfinite(xyz).all(axis=1)&(xyz[:,0]>0)
    marked=xyz[finite].astype('<f4').tobytes()
    if max_depth==0:return marked,b'',0
    valid=finite&(xyz[:,0]<=max_depth+.001)
    # Retain the native clearing path's eight finite-anchor requirement.
    no_return=np.isposinf(xyz[:,0])if valid.sum()>=8 else np.zeros(len(xyz),dtype=bool)
    cleared=np.concatenate([xyz[valid],rays[no_return]*max_depth])
    return marked,cleared.astype('<f4').tobytes(),int(no_return.sum())
