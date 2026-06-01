# Single highest-confidence person detection (single-target follow).
# All bbox coords are normalized [0,1] relative to image dimensions.
bool detected
float32 bbox_cx
float32 bbox_cy
float32 bbox_w
float32 bbox_h
float32 distance      # meters, -1.0 if unknown
float32 confidence    # [0,1]
builtin_interfaces/Time stamp
