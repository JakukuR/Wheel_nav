"""20 Hz follow PID with bounded sector avoidance and hard stop conditions."""
import math


class FollowController:
    def __init__(self, distance=1.4, max_linear=.20, max_angular=.45,
                 prediction_linear=.08, emergency_distance=.60):
        self.distance = distance
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.prediction_linear = prediction_linear
        self.emergency_distance = emergency_distance
        self.reset()

    def reset(self):
        self.integral = 0.
        self.previous_error = 0.
        self.previous_angle = 0.

    def command(self, target_xy, grid, dt, predicting=False):
        if target_xy is None or grid is None:
            self.reset()
            return 0., 0., 'no_target_or_depth'
        if grid.front_distance() < self.emergency_distance:
            self.reset()
            return 0., 0., 'emergency_obstacle'
        x, y = float(target_xy[0]), float(target_xy[1])
        distance = math.hypot(x, y)
        angle = math.atan2(y, x)
        if distance < .50:
            self.reset()
            return 0., 0., 'target_too_close'
        dt = max(.02, min(.2, dt))
        error = distance - self.distance
        self.integral = max(-.5, min(.5, self.integral + error * dt))
        linear = .45*error + .06*self.integral + .04*(error-self.previous_error)/dt
        angular = 1.2*angle + .08*(angle-self.previous_angle)/dt
        self.previous_error = error
        self.previous_angle = angle
        linear = max(0., min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))
        if abs(angle) > .55:
            linear = 0.
        if predicting:
            linear = min(linear, self.prediction_linear)
            angular = max(-.25, min(.25, angular))
        left = grid.sector_blocked(1)
        right = grid.sector_blocked(-1)
        if left and right:
            return 0., 0., 'both_sectors_blocked'
        if left and angle > 0 or right and angle < 0:
            linear = 0.
            angular = -.20 if left else .20
        # Inspect the intended forward corridor in the inflated grid.
        if linear > 0 and (grid.collision_at(.65, 0.) or grid.collision_at(1., 0.)):
            linear = 0.
        return linear, angular, 'predicting' if predicting else 'tracking'
