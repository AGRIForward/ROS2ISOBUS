# TIM auxiliary valve command (AEF 023 / TIM auxiliary valve function)
uint8 valve_number
float32 flow_percent  # signed % flow: +extend, -retract
bool enable           # true=request control, false=release control
