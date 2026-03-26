# TIM rear PTO status (AEF 023 / TIM rear PTO function)
float64 rpm              # PTO shaft speed (RPM)
uint8 automation_status  # TIM SLOT automation status (10.2.2)
bool valid               # true when rear PTO status has been received
bool ok                  # true when no fault is indicated
bool active              # true when automation is active
bool limited_high        # true when high-limit active status is indicated
bool limited_low         # true when low-limit active status is indicated
bool fault               # true when automation fault is indicated
