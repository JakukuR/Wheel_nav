"""Quality policy for the long-lived follow template, not BoxMOT association.

BoxMOT still extracts/uses all person embeddings. Low-quality crops must not
silently overwrite the robot's selected-person gallery.
"""
import numpy as np


def feature_quality(row,objects,width,height):
    if row.get('label')!='person':return 'not_person'
    if row.get('confidence',0)<.5:return 'low_confidence'
    b=np.asarray(row['bbox_xyxy'],float);x1,y1,x2,y2=b
    if x2-x1<32 or y2-y1<64:return 'crop_too_small'
    if sum((x1<=1,y1<=1,x2>=width-1,y2>=height-1))>=2:return 'crop_clipped'
    area=(x2-x1)*(y2-y1)
    if area>width*height*.7:return 'crop_too_large'
    for other in objects:
        if other['track_id']==row['track_id'] or other['label']!='person':continue
        a=other['bbox_xyxy'];other_area=(a[2]-a[0])*(a[3]-a[1])
        overlap=max(0,min(x2,a[2])-max(x1,a[0]))*max(0,min(y2,a[3])-max(y1,a[1]))
        if overlap/max(1,min(area,other_area))>.25:return 'crop_overlap'
    return 'feature_ready'
