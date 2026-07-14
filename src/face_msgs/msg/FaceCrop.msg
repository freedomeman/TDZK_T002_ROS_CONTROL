# 主目标人脸裁剪图（低频 ~8Hz，用于表情识别）
std_msgs/Header header
bool has_face                     # 是否有主目标；false 时 face_crop 为空
sensor_msgs/Image face_crop       # 裁剪后的人脸图（带 padding），bgr8