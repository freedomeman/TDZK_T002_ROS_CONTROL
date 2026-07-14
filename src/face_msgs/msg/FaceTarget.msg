# 主目标 3D 坐标（实时高频，用于电机跟随）
# 相机坐标系（相机光心为原点）：
#   X 轴：深度方向（朝向物体为正）
#   Y 轴：屏幕水平方向（右为正）
#   Z 轴：屏幕垂直方向（上为正）
std_msgs/Header header
bool has_target                   # 是否检测到主目标
geometry_msgs/Point center        # (X, Y, Z) 米；无效时全 0