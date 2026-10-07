# detection_id は同じ画像内だけで有効な識別子．追跡 ID ではない．
uint32 detection_id
int32 class_id
string class_name
float32 confidence
BoundingBox bounding_box
# 元画像上の領域と，その領域に切り出した mono8 マスク（背景 0，物体 255）．
sensor_msgs/RegionOfInterest roi
sensor_msgs/Image mask
