#!/usr/bin/env python3
"""Audit extracted samples with Kalibr's actual AprilGrid detector; no solver."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import multiprocessing
from pathlib import Path
import time
import cv2
import numpy as np
import aslam_cv as acv
import aslam_cameras_april as april


def detect(job):
    root,row=job
    cv2.setNumThreads(1)
    image=cv2.imread(str(Path(root)/row['file']),cv2.IMREAD_GRAYSCALE)
    h,w=image.shape
    if hashlib.sha256(image.tobytes()).hexdigest()!=row['grayscale_sha256']:
        raise ValueError('PNG bytes differ from exported source grayscale')
    # Detection-only dummy geometry, matching the official extraction API;
    # geometric outlier filtering is disabled as in CameraCalibrator.py.
    projection=acv.DistortedPinholeProjection(1000.,1000.,w/2,h/2,w,h,
        acv.RadialTangentialDistortion(0.,0.,0.,0.))
    geometry=acv.DistortedPinholeCameraGeometry(projection)
    options=april.AprilgridOptions();options.minTagsForValidObs=7;options.blackTagBorder=2
    grid=april.GridCalibrationTargetAprilgrid(6,6,.0325,.3,options)
    detector_options=acv.GridDetectorOptions();detector_options.filterCornerOutliers=False
    detector=acv.GridDetector(geometry,grid,detector_options)
    ok,obs=detector.findTargetNoTransformation(acv.Time(1.),image)
    indices=np.asarray(obs.getCornersIdx()).reshape(-1).astype(int).tolist()
    pixels=np.asarray(obs.getCornersImageFrame()).reshape(-1,2).tolist()
    return dict(row,accepted=bool(ok),corner_indices=indices,corners_xy=pixels,
        tag_ids=sorted({i//4 for i in indices}),corner_count=len(indices))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();args.output.mkdir(exist_ok=False)
    manifest=json.loads((args.input/'manifest.json').read_text())
    started=time.monotonic();results=[]
    cv2.setNumThreads(1)
    with ProcessPoolExecutor(max_workers=2,mp_context=multiprocessing.get_context('fork')) as pool:
        for number,row in enumerate(pool.map(detect,[(str(args.input),r) for r in manifest['frames']],chunksize=4)):
            results.append(row)
            if (number+1)%60==0:print('Kalibr checked {}/{} images, {:.0f}s'.format(number+1,len(manifest['frames']),time.monotonic()-started),flush=True)
    lookup={r['file']:r for r in results};cameras={};pairs={}
    for eye in ('left','right','rgb'):
        frames=[r for r in results if r['eye']==eye];accepted=[r for r in frames if r['accepted']]
        cells=np.zeros((3,4),int)
        for r in accepted:
            x,y=np.asarray(r['corners_xy']).mean(axis=0)
            cells[min(2,int(y/r['height']*3)),min(3,int(x/r['width']*4))]+=1
        cameras[eye]=dict(sampled_frames=len(frames),accepted_frames=len(accepted),
            max_observed_tags=max((len(r['tag_ids']) for r in frames),default=0),
            max_corners=max((r['corner_count'] for r in frames),default=0),
            accepted_board_centroid_grid_3x4=cells.tolist(),
            accepted_times_s=[round((r['stamp']-manifest['start_stamp'])/1e9,2) for r in accepted])
    for a,b in (('left','right'),('left','rgb'),('right','rgb')):
        rows=[];seen=set()
        for group in manifest['groups']:
            key=(group[a],group[b])
            if key in seen:continue
            seen.add(key);ra,rb=lookup[key[0]],lookup[key[1]]
            common=sorted(set(ra['corner_indices'])&set(rb['corner_indices']))
            rows.append(dict(t_s=round((min(ra['stamp'],rb['stamp'])-manifest['start_stamp'])/1e9,3),
                skew_ms=abs(ra['stamp']-rb['stamp'])/1e6,
                both_accepted=ra['accepted'] and rb['accepted'],common_corners=len(common),
                common_tag_ids=sorted({i//4 for i in common})))
        valid=[r for r in rows if r['both_accepted'] and r['skew_ms']<=25 and r['common_corners']>0]
        pairs[a+'--'+b]=dict(sampled_unique_pairs=len(rows),
            accepted_common_corner_pairs_within_25ms=len(valid),
            pairs_with_ge20_common_corners=sum(r['common_corners']>=20 for r in valid),
            max_common_corners=max((r['common_corners'] for r in valid),default=0),
            valid_times_s=[r['t_s'] for r in valid],details=rows)
    report=dict(method='official Kalibr AprilGrid extraction, minTags=7, blackTagBorder=2',
        sampling_period_s=manifest['period_s'],pixel_hashes_match=True,solver_started=False,
        camera_quality=cameras,pair_quality=pairs,elapsed_s=time.monotonic()-started,
        limitations='Sparse samples; repeated stationary frames are not independent poses. Host timestamps are not exposure synchronization. No intrinsics, extrinsics or reprojection accuracy have been estimated.')
    (args.output/'detections.json').write_text(json.dumps(results,indent=2))
    (args.output/'report.json').write_text(json.dumps(report,indent=2))
    strips=[]
    for eye in ('left','right','rgb'):
        chosen=[]
        for r in sorted((r for r in results if r['eye']==eye),key=lambda r:r['corner_count'],reverse=True):
            if all(abs(r['stamp']-other['stamp'])>=20e9 for other in chosen):chosen.append(r)
            if len(chosen)==4:break
        tiles=[]
        for r in chosen:
            image=cv2.imread(str(args.input/r['file']))
            for x,y in r['corners_xy']:cv2.circle(image,(round(x),round(y)),3,(0,220,0),-1)
            tile=np.full((305,480,3),24,np.uint8);tile[35:]=cv2.resize(image,(480,270),interpolation=cv2.INTER_AREA)
            text='{} t={:.1f}s corners={} ok={}'.format(eye,(r['stamp']-manifest['start_stamp'])/1e9,r['corner_count'],r['accepted'])
            cv2.putText(tile,text,(7,24),cv2.FONT_HERSHEY_SIMPLEX,.46,(240,240,240),1);tiles.append(tile)
        strips.append(np.concatenate(tiles,axis=0))
    cv2.imwrite(str(args.output/'official_samples.jpg'),np.concatenate(strips,axis=1))
    summary=dict(report);summary['pair_quality']={k:{a:b for a,b in v.items() if a not in ('details','valid_times_s')} for k,v in pairs.items()}
    summary['camera_quality']={k:{a:b for a,b in v.items() if a!='accepted_times_s'} for k,v in cameras.items()}
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':main()
