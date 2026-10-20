# Vehicle-mounted navigation reference for moving-platform initialization.
# Data only: this topic does not authorize arming or modify estimator states.
# All timestamps are in the publishing flight controller's monotonic boot clock,
# in microseconds. Zero means unavailable; never substitute transmit/receive time
# for a sample timestamp. Remote clock conversion belongs to the future receiver.

uint64 timestamp                       # Reference publication time
uint64 timestamp_attitude_sample       # vehicle_attitude.timestamp_sample
uint64 timestamp_velocity_sample       # vehicle_local_position.timestamp_sample
uint64 timestamp_angular_velocity_sample # vehicle_angular_velocity.timestamp_sample

float32[4] q                           # Hamilton quaternion (w,x,y,z), body FRD to earth NED
float32[3] velocity                    # North, east, down velocity (m/s), at the source navigation reference point
float32[3] angular_velocity            # Bias-corrected body FRD angular velocity (rad/s)

# Invalid quantities must not be consumed, even if their numeric values are finite.
# Publishers clear the corresponding bits for missing, stale or rejected data.
# Tilt and heading are separate: a finite quaternion does not establish heading quality.
uint8 VALID_TILT = 1
uint8 VALID_HEADING = 2
uint8 VALID_HORIZONTAL_VELOCITY = 4
uint8 VALID_VERTICAL_VELOCITY = 8
uint8 VALID_ANGULAR_VELOCITY = 16
uint8 validity_flags                   # Bitmask of VALID_*; zero means no valid reference

uint8 quat_reset_counter               # Copied from vehicle_attitude; wraps modulo 256
uint8 vxy_reset_counter                # Copied from vehicle_local_position; wraps modulo 256
uint8 vz_reset_counter                 # Copied from vehicle_local_position; wraps modulo 256
uint8 estimator_instance               # Selected estimator instance; 255 if unknown

# Position, alignment completion, arming permission and clock-sync status are
# intentionally excluded. Consumers must invalidate history on source reboot or
# estimator switch. Reset counters alone cannot identify a flight-controller reboot.
