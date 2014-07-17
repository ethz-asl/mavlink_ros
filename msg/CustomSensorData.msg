Header header

# Differential pressure, already temperature-compensated
float32 dbaro_pres_pa

# Velocity calculation from differential pressure sensor in m/sec
float32 dbaro_velo_ms

# Ambient temperature in degrees celsius
float32 amb_temp_celsius

# Power board voltage sensor reading in volts
float32 adc121_vspb_volt

# Power board current sensor reading in amps
float32 adc121_cspb_amp

# Board current sensor 1 reading in amps
float32 adc121_cs1_amp

# Board current sensor 2 reading in amps
float32 adc121_cs2_amp

# MPPT1 voltage + current
float32 mppt1_volt
float32 mppt1_amp 

# MPPT2 voltage + current
float32 mppt2_volt 
float32 mppt2_amp

# MPPT3 voltage + current
float32 mppt3_volt
float32 mppt3_amp

# MPPT1-3 pwm
uint16 mppt1_pwm
uint16 mppt2_pwm
uint16 mppt3_pwm

# MPPT1-3 status
uint8 mppt1_status
uint8 mppt2_status
uint8 mppt3_status
