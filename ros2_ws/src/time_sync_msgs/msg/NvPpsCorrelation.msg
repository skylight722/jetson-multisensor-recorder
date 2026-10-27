# PTP event time. This is the timestamp used as the common time axis.
std_msgs/Header header

# Sequential index assigned by this ROS publisher.
uint64 sample_index

# NvPPS event information.
uint32 event_number
uint64 host_realtime_ns
uint64 tsc_ns
uint64 ptp_ns
uint64 secondary_ptp_ns
uint64 irq_latency_ns
uint64 tsc_resolution_ns
uint32 event_mode
uint32 tsc_mode

# Signed offset observed at this anchor.
int64 ptp_minus_tsc_ns
