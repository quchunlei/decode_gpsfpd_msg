Header header

int32 GPSWeek
float64 GPSTime

#orientation
float64 Heading
float64 Pitch
float64 Roll

#quaternion sRot

#position
float64 Lattitude
float64 Longitude
float64 Altitude
float64 dX_e
float64 dY_d
float64 dX
float64 dY
float64 dtheta

#speed
float64 VEast
float64 VNorth
float64 VUp

float64 Baseline
int16 nSatelitesNum1
int16 nSatelitesNum2
int8 Status
int8[3] Reserved 


int64 ReceivedMSGTimeFlags
int64 SendMSGTimeFlags
int64 ImageTimeFlags

string INS_state
string POS_state

int64 FrameNo 
