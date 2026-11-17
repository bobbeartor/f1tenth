# Observation only. No brake/drive commands are generated.
# Header is the source exposure timestamp, NOT publication time.
std_msgs/Header header
uint8 UNKNOWN=0
uint8 RED=1
uint8 GREEN=2
uint8 state
float32 detection_score
uint64 source_generation
float32 inference_ms
float32 processing_ms
float32 capture_age_ms
