# The stamp currently assigned to the corresponding Image message.
std_msgs/Header header

# Camera identifier, such as camera_front or cam1.
string camera_name

# V4L2 capture metadata.
uint32 sequence
uint32 v4l2_flags
uint32 v4l2_timestamp_clock_flags
uint32 v4l2_timestamp_source_flags
bool v4l2_buffer_error
uint64 raw_v4l2_timestamp_ns

# Host clock samples taken immediately after VIDIOC_DQBUF.
uint64 dqbuf_realtime_before_ns
uint64 dqbuf_monotonic_ns
uint64 dqbuf_monotonic_raw_ns
uint64 dqbuf_boottime_ns
uint64 dqbuf_tsc_cycles
uint64 dqbuf_tsc_ns
uint64 dqbuf_tsc_frequency_hz
uint64 dqbuf_realtime_after_ns

# Time taken to collect the clock sample set.
uint64 dqbuf_sample_span_ns

# Values used or observed by the existing timestamp conversion.
int64 sampled_realtime_minus_monotonic_raw_ns
int64 sampled_tsc_minus_monotonic_raw_ns
uint64 startup_tsc_offset_ns
int64 configured_timestamp_offset_ns
int64 current_header_stamp_ns

# Capture/pipeline state.
bool publish_requested
bool image_enqueued
