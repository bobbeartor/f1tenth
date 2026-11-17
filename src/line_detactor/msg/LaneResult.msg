# Same timestamp and source frame_id as the input BEV. Pixel mapping is explicit below.
std_msgs/Header header
# Optional camera identity assigned by the tunnel selector (RGB or IR).
# Header capture times remain original and may overlap across cameras.
# Empty for ordinary/standalone detector output.
string lane_source
# Per-frame performance metadata. detector_input_received_stamp is when the
# line detector received the BEV image; detector_result_ready_stamp is taken
# immediately before publishing this result. All durations exclude GUI work.
builtin_interfaces/Time detector_input_received_stamp
builtin_interfaces/Time detector_result_ready_stamp
string engine_precision
uint64 detector_sequence
uint64 detector_queue_nanoseconds
uint64 h2d_preprocess_nanoseconds
uint64 pure_inference_nanoseconds
uint64 label_export_nanoseconds
uint64 backend_postprocess_nanoseconds
uint64 lane_geometry_nanoseconds
uint64 result_message_build_nanoseconds
uint64 detector_total_compute_nanoseconds
uint8 NONE=0
uint8 LEFT_ONLY=1
uint8 RIGHT_ONLY=2
uint8 BOTH=3
# State means retained label presence, not a validated single curve per side.
uint8 state
uint8 BORDER_ONLY=1
uint8 processing_mode
uint32 source_width
uint32 source_height
uint32 padding_left
uint32 padding_right
# Original pixel: u_source = u_result - padding_left; v_source = v_result.
# Negative/out-of-source coordinates are inferred, not newly observed camera pixels.
# The latency-sensitive control result intentionally carries no BGR, label or
# mask images. Debug visualization remains available on result_image_topic.
# True iff the independent model stop-line output contains a positive pixel;
# this is diagnostic and is not itself a vehicle stop command.
bool stop_line_present
# Along-path distance from front axle to near stop-line edge at header.stamp.
# NaN when unavailable. Consumers must check finiteness, not default to zero.
float32 stop_line_distance_m

# Actual retained model-supported boundary component arc, longest near-vehicle
# component over either side. Excludes bridges/inferred centerline/far islands.
# Valid zero means a processed frame with no eligible observed component.
# False means unavailable; consumers must not interpret it as lane loss.
bool observed_lane_length_valid
float32 observed_lane_length_m

# Yellow center path, derived independently of the internal display masks.
# Pixels in the padded image, ordered from near vehicle to far along the path.
# valid only means a geometric path exists, NOT a validated driving command.
bool centerline_valid
bool centerline_sample_limit_reached
geometry_msgs/Point32[] centerline_points
uint8 CENTER_SINGLE=1
uint8 CENTER_PAIRED=2
uint8 CENTER_GAP=3
uint8 CENTER_OUTER=4
# Same length as points. Local construction support, not a probability.
# Short graph edges and subsequent smoothing can mix neighboring support.
uint8[] centerline_support
# Physical extent of the unpadded source used for centerline geometry.
float32 centerline_bev_width_m
float32 centerline_bev_height_m
