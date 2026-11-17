#!/bin/sh
# Jetson pwm-fan: 255 is 100% duty. Do not change the GPU or power mode.
set -eu
state=/run/wla-fan-full-speed.policy
if [ "${1:-}" = restore ]; then
    if [ -f "$state" ]; then
        read -r zone original < "$state"
        printf '%s\n' "$original" > "$zone/policy"
        rm -f "$state"
    fi
    exit 0
fi
attempt=0
while [ "$attempt" -lt 30 ]; do
    for pwm in /sys/devices/platform/pwm-fan/hwmon/hwmon*/pwm1; do
        [ -w "$pwm" ] || continue
        # Only take over the fan-only thermal zone; leave CPU/GPU throttling
        # and critical-temperature handling enabled.
        for zone in /sys/class/thermal/thermal_zone*; do
            [ "$(cat "$zone/type")" = tj-thermal ] || continue
            count=0
            for cooling in "$zone"/cdev*; do
                [ -L "$cooling" ] || continue
                [ "$(cat "$cooling/type")" = pwm-fan ] || exit 1
                count=$((count + 1))
            done
            [ "$count" -gt 0 ] || exit 1
            if [ ! -f "$state" ]; then
                printf '%s %s\n' "$zone" "$(cat "$zone/policy")" > "$state"
            fi
            printf 'user_space\n' > "$zone/policy"
        done
        printf '255\n' > "$pwm"
        [ "$(cat "$pwm")" = 255 ] || exit 1
        echo "Fan full-speed duty verified: $pwm = 255"
        exit 0
    done
    attempt=$((attempt + 1))
    sleep 1
done
echo 'Jetson pwm-fan control was not available after 30 seconds' >&2
exit 1
