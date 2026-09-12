string name
string stage            # carding / drawing1 / drawing2
string state            # idle / processing / waiting / blocked
float64 x
float64 y
int32 input_count       # 待加工料
int32 output_count      # 已完工料
float64 busy_ratio
string waiting_station   # 该机台的待加工工位名
string finished_station  # 该机台的完工工位名
