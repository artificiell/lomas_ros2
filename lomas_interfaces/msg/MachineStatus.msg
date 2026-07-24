# lomas_interfaces/msg/MachineStatus
# --------------------------------------------------------------------------------------
bool    is_synced        # False if the last exchange with the machine could not be
                         # confirmed (timeout, reset, or unexpected response) —
                         # a signal that subsequent status may be stale/unreliable
bool    machine_moving   # True from the moment a move is accepted until the
                         # position-poll timer observes the machine go Idle
uint8   error_nr         #  0 = ok, 1 = serial error/timeout, 2 = out-of-range
                         # command rejected, 98 = connection/reinit failure
uint16  interval         # Cultivation interval
float32 x
float32 y
float32 z
