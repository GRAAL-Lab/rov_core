float64 time           # Time of update in UNIX time with nanoSec precision (sec.nsec)
uint8 gpsfixmode
uint64 flags
float64 latitude       # Latitude in deg (valid if mode >= 2) 
float64 longitude      # Longitude in deg (valid if mode >= 2) 
float64 altitude       # Altitude in meters (valid if mode == 3) 
float64 track          # Course made good in rad (relative to true north) 
float64 speed          # Speed over ground, meters/sec 
float64 climb          # Vertical speed, meters/sec 

float64 err            # spherical position error, 95% confidence (meters)  
float64 err_time       # Expected time uncertainty 
float64 err_latitude   # Latitude position uncertainty, meters 
float64 err_longitude  # Longitude position uncertainty, meters 
float64 err_altitude   # Vertical position uncertainty, meters 
float64 err_track      # Track uncertainty, degrllc 
float64 err_speed      # Speed uncertainty, meters/sec 
float64 err_climb      # Vertical speed uncertainty 

float64 xdop
float64 ydop
float64 gdop           # Total (positional-temporal) dilution of precision 
float64 pdop           # Positional (3D) dilution of precision 
float64 hdop           # Horizontal dilution of precision 
float64 vdop           # Vertical dilution of precision 
float64 tdop           # Temporal dilution of precision 

uint8 MODENOTSEEN=0  # mode update not seen yet 
uint8 MODENOFIX=1    # none 
uint8 MODE2D=2        # good for latitude/longitude 
uint8 MODE3D=3        # good for altitude/climb too 
