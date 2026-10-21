from wla_r680_navigation_py.identity_features import feature_quality

def box(i,b,score=.9):return dict(track_id=i,label='person',confidence=score,bbox_xyxy=b)

def test_template_quality_separate_from_native_tracking():
    row=box(1,[20,20,100,300]);other=box(2,[200,20,280,300])
    assert feature_quality(row,[row,other],640,480)=='feature_ready'
    assert feature_quality(box(1,[20,20,100,300],.3),[other],640,480)=='low_confidence'
    overlap=box(3,[30,30,105,305])
    assert feature_quality(row,[row,overlap],640,480)=='crop_overlap'
    assert feature_quality(box(1,[0,0,100,300]),[],640,480)=='crop_clipped'
    assert feature_quality(box(1,[10,10,20,40]),[],640,480)=='crop_too_small'
