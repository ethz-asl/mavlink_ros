Header header

uint32 time_boot_ms # Timestamp (milliseconds since system boot)
int32 lat # Latitude, expressed as meters * 10^7
int32 lon # Longitude, expressed as meters * 10^7
int32 alt # Altitude, expressed as meters * 1000 (millimeters) above MSL
int32 relative_alt # Altitude above ground, expressed as meters * 1000 (millimeters)
int16 vx # Ground X speed (latitude), expressed as meters per second * 100
int16 vy # Ground Y speed (longitude), expressed as meters per second * 100
int16 vz # Ground Z speed (altitude), expressed as meters per second * 100
uint16 hdg # Compass heading, expressed in degrees * 100
