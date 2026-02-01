uint8 cached  # frame_cached number
int64 stamp_shot  # stamp of shooting frame, steady clock, ns
int64 stamp_get  # stamp after get frame, steady clock, ns
int64 stamp_newest  # stamp of newest frame, steady clock, ns
uint8[] frame  # video frame
uint8 et_enc  # encoding elapsed time, ms
int64 stamp_sen  # stamp before send frame, steady clock, ms
uint8 et_dec  # decoding elapsed time, ms
float32 net_speed  # network speed