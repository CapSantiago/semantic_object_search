"""Conservative same-frame supplementation of the unchanged primary detector."""
from math import isfinite
from .ball_geometry import BALL_SUBTYPES


def auxiliary_candidate(label, box, primary, enabled):
    """Return a canonical label/COCO ID, or reject unsupported/duplicate output.

    Primary boxes retain priority when the same category overlaps by >=.5 IoU.
    Different visible instances remain separate; no room or object truth enters.
    """
    category='sports ball' if label in BALL_SUBTYPES else label
    ids={'book':73,'sports ball':32}
    if category not in ids or category not in enabled:return None
    if len(box)!=4 or not all(isfinite(v) for v in box):return None
    x1,y1,x2,y2=box;area=max(0,x2-x1)*max(0,y2-y1)
    if area<=0:return None
    for name,other in primary:
        if name!=category:continue
        a,b,c,d=other
        intersection=max(0,min(x2,c)-max(x1,a))*max(0,min(y2,d)-max(y1,b))
        union=area+max(0,c-a)*max(0,d-b)-intersection
        if union>0 and intersection/union>=.5:return None
    return category,ids[category]
