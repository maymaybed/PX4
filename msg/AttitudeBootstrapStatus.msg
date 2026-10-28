# An initialization result, including the received and actually applied attitude.
# ACCEPTED means applied by the EKF, rather than merely received by the transport.
uint64 timestamp
uint64 timestamp_sample
uint64 request_id
uint64 source_timestamp
uint64 timestamp_applied
uint32 timestamp_uncertainty_us
float32[4] q
float32[4] q_applied
float32[3] variance
float32[3] variance_applied # precision actually used, including clock uncertainty
uint8 frame
uint8 child_frame
uint8 source_system
uint8 source_component
uint8 target_system
uint8 target_component
uint8 estimator_instance
uint8 result

uint8 ACCEPTED = 0
uint8 DISABLED = 1
uint8 WRONG_SOURCE = 2
uint8 ARMED = 3
uint8 INVALID_FRAME = 4
uint8 INVALID_QUATERNION = 5
uint8 INVALID_VARIANCE = 6
uint8 INACCURATE = 7
uint8 STALE = 8
uint8 FUTURE = 9
uint8 ALREADY_INITIALIZED = 10
uint8 BUSY = 11
uint8 WAITING = 12
uint8 UNSUPPORTED = 13
uint8 TIME_UNCERTAIN = 14
uint8 NO_IMU = 15
uint8 PROTOCOL_ERROR = 16
uint8 NO_ATTITUDE = 17
uint8 WRONG_TARGET = 18

uint8 ORB_QUEUE_LENGTH = 16
