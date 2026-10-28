# One-shot attitude initialization request, validated by the MAVLink bootstrap service.
# Attitude is Hamilton w,x,y,z, rotating body FRD to earth NED (true north).
# Source and receiver boot clocks are independent. timestamp_sample is mapped to
# the receiver clock by a bounded request/response exchange, never by subtraction
# of unrelated boot timestamps. No position/velocity observations are supplied.
uint64 timestamp
uint64 timestamp_sample
uint64 request_id
uint64 source_timestamp
uint32 timestamp_uncertainty_us
float32[4] q
float32[3] variance # conservative earth tangent attitude variance [rad^2]
uint8 frame
uint8 child_frame
uint8 source_system
uint8 source_component
uint8 target_system
uint8 target_component

uint8 ORB_QUEUE_LENGTH = 8
