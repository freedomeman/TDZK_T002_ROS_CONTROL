# 单次表情识别结果（针对主目标）
std_msgs/Header header

int32 track_id                  # 对应上游 face 的 track_id（暂时 -1）
bool  has_face                  # 本帧是否有主目标（无则其它字段无意义）

# ---- 单帧原始结果 ----
uint8 EMO_NEUTRAL  = 0
uint8 EMO_HAPPY    = 1
uint8 EMO_SAD      = 2
uint8 EMO_SURPRISE = 3
uint8 EMO_FEAR     = 4
uint8 EMO_DISGUST  = 5
uint8 EMO_ANGER    = 6

uint8   raw_emotion_id          # 单帧 argmax 结果
float32 raw_confidence          # 单帧 softmax 最大值
float32[] probabilities         # 7 维概率（softmax 后）

# ---- 平滑后稳定结果 ----
uint8   stable_emotion_id       # 经过窗口投票 + 阈值的稳定表情
float32 stable_confidence       # 稳定表情的平均置信度
bool    stable_changed          # 相对上一次是否发生了变化（下游据此触发）