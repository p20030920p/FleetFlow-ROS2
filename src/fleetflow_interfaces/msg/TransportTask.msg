# 一次搬运任务：把 material_type 从 source 送到 dest
int32 task_id
int32 priority          # 越小越优先
string material_type    # empty / green / yellow / red
string source_name
string dest_name
float64 source_x
float64 source_y
float64 dest_x
float64 dest_y
string status           # pending / assigned / running / done / failed
int32 robot_id          # -1 = 未分配
