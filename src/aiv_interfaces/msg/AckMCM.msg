ItsPduHeader its_header
uint16 generation_time #timestamp

#basic container 
StationType station_type 
float64[] current_position

#Destinater container
StationType station_type_destinater 
uint32 station_id_destinater

#ManeuverContainer
ManeuverContainer maneuver

bool ack_mcm_response
