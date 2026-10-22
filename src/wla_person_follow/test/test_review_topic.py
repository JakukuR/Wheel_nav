from pathlib import Path
import ast
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[2]
def test_presets_use_detector_published_status_topic():
    tree=ast.parse((ROOT/'wla_r680_navigation/scripts/nav_yolo_detector').read_text())
    published=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(getattr(t,'id',None)=='HEALTH_TOPIC' for t in n.targets))
    for config in (ROOT/'wla_person_follow/config').glob('sim_follow*.yaml'):
        assert yaml.safe_load(config.read_text())['person_follow']['ros__parameters']['detection_status_topic']==published

def test_startup_rejects_mismatched_detection_status_topic():
    from wla_person_follow.follow_config import validate_detection_status_topic
    with pytest.raises(ValueError,match='detection_status_topic_mismatch'):
        validate_detection_status_topic('/r680/nav/perception/status')
    validate_detection_status_topic('/r680_nav/perception/status')
