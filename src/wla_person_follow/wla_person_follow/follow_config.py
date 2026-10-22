"""ROS-free validation for the two supported person-follow presets."""
NEW_PRESET_FLAGS = (
    'geometry_binding_v2',
    'front_check_truncation_enabled',
    'identity_supervisor_v2',
    'lost_task_release',
    'mask_binding_v2',
    'reverse_policy_v2',
    'diagnostic_v2',
)

PENDING_CALIBRATION_PARAMETERS = (
    'front_check_max_m',
    'front_check_target_offset_m',
    'binding_missing_grace_s',
    'binding_jump_radius_m',
    'binding_jump_growth_mps',
    'binding_jump_radius_max_m',
    'enrollment_speed_cap_mps',
    'enrollment_timeout_s',
    'suspect_negative_frames',
    'suspect_recovery_frames',
    'suspect_timeout_s',
    'neighbor_ambiguity_radius_m',
    'neighbor_clear_hold_s',
    'active_search_s',
    'target_release_s',
    'view_expansion_similarity',
    'view_expansion_neighbor_radius_m',
    'view_expansion_min_interval_s',
    'view_expansion_initial_slots',
)


def validate_follow_preset(parameters):
    """Accept only a complete legacy or complete first-batch configuration."""
    marker = parameters.get('follow_preset')
    if marker not in ('compatibility', 'first_batch_v2'):
        raise ValueError('invalid_follow_preset_configuration')
    enabled = [name for name in NEW_PRESET_FLAGS if parameters.get(name)]
    if marker == 'compatibility' and enabled:
        raise ValueError('invalid_follow_preset_configuration')
    if marker == 'first_batch_v2':
        if any(not parameters.get(name) for name in NEW_PRESET_FLAGS):
            raise ValueError('invalid_follow_preset_configuration')
        if (not parameters.get('lost_task_release')
                or not parameters.get('reid_enabled')
                or not parameters.get('reid_model_id')):
            raise ValueError('invalid_follow_preset_configuration')
    return marker

def target_freshness_decision(*, source_fresh, geometry_binding_v2,
                              binding_state, observation_age_s,
                              max_sensor_age_s, missing_grace_s):
    """Return ``(allowed, reason)`` for target output freshness.

    The grace extends only the existing target-observation gate. Depth, wheel,
    collision, permission, and result-TTL predicates remain separate.
    """
    if source_fresh:
        return True, 'source_observations_fresh'
    if (geometry_binding_v2 and binding_state == 'bound_coasting'
            and observation_age_s is not None):
        limit = float(max_sensor_age_s) + float(missing_grace_s)
        if 0. <= observation_age_s <= limit + 1e-9:
            return True, 'geometry_binding_missing_grace'
        return False, 'target_observation_missing_grace_expired'
    return False, 'target_observations_stale'
