"""Configuration contract for the single tracker-led follow chain."""
PENDING_CALIBRATION_PARAMETERS = (
    'binding_jump_radius_m','binding_jump_growth_mps','binding_jump_radius_max_m',
    'neighbor_ambiguity_radius_m','reid_similarity','reid_margin','reid_confirm_frames',
    'target_ghost_clear_radius_m','target_ghost_clear_ttl_s','search_arrival_m',
)


def validate_follow_config(parameters):
    if parameters.get('follow_preset') != 'tracker_follow':
        raise ValueError('retired_follow_preset: use current config/sim_follow.yaml')
    if not parameters.get('target_mask_enabled', True):
        raise ValueError('tracker_follow_requires_target_filter')
    if parameters.get('reid_enabled') and not parameters.get('reid_model_id'):
        raise ValueError('reid_model_id_required')
    return 'tracker_follow'


def validate_detection_status_topic(topic):
    if topic != '/r680_nav/perception/status':
        raise ValueError('detection_status_topic_mismatch: expected /r680_nav/perception/status')
