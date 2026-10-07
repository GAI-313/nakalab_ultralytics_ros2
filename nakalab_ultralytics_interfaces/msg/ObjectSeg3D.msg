uint32 detection_id
int32 class_id
string class_name
float32 confidence
bool valid
string reason
# 可視表面の外れ値除去後の重心と，出力座標系に平行な外接箱（m）．
# valid=False では座標・寸法は NaN．物体固有の回転姿勢は推定しない．
geometry_msgs/Point centroid
geometry_msgs/Point box_center
geometry_msgs/Vector3 size
uint32 valid_points
