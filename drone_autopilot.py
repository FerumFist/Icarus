"""
The objective of this module is to be able to do following modes:
 # EARLY
 - follow given heading within set altitude - GA autopilot
 - follow pre-determined waypoints consisting of a lat, lon and altitude
 - pitch, roll and yaw limiter from gyro information, no more than 20 dungarees in any direction
 - RTTH - opposite heading to that of start, 2 meters above start altitude, lowest possible speed (stall + 10%)
 # LATER
 - autothrottle mode - pitch dependent | keep IAS above minimums etc... (Need IAS sensor, pitot tube)
    - alternatively GS autothrottle, limited to 75% maximum and stall speed + 10% minimum
 - stabilization mode - easy fly

"""
def pitch_down():
    # pitch down shite
def pitch_up():
    # pitch down shite
def roll_left():
    # roll left
def roll_right():
    # roll right
def yaw_right():
    # yaw right
def yaw_left():
    # yaw left
def hdg():
    # hdg hold
def alt():
    # alt hold
def ap_routine(REQ_LAT, REQ_LON, REQ_ALT, REQ_HDG):

        MAX_ALT = 1500
        MIN_ALT = 50
        MAX_CLIMB_MPS = 30
        MAX_DESC_MPS  = 30

        CURR_ALT = gps.alt()
        CURR_HDG = gps.hdg()
        # chk if commanded values within bounds, if not, disengage, if yes, continue
        if not (REQ_ALT < MAX_ALT and REQ_ALT > MIN_ALT):
            raise BadAltitude

        CHG_ALT = REQ_ALT - CURR_ALT
        # AP CLIMB
        if CHG_ALT > 0:
            if CHG_ALT > 100:
                AUTO_PITCH = 10
            if CHG_ALT < 100:
                AUTO_PITCH = 10
                AUTO_PITCH_MULT = (CHG_ALT / MAX_CLIMB_MPS / 10) * 2.5  # anywhere betwween 50% and 0.01%
                AUTO_PITCH = AUTO_PITCH * AUTO_PITCH_MULT
        # AP DESCEND
        if CHG_ALT < 0:
            if CHG_ALT > -100:
                AUTO_PITCH = 10
            if CHG_ALT < -100:
                AUTO_PITCH = -10
                AUTO_PITCH_MULT = (CHG_ALT / MAX_CLIMB_MPS / 10) * 2.5  # anywhere betwween 50% and 0.01%
                AUTO_PITCH = AUTO_PITCH * AUTO_PITCH_MULT
        # AP HDG CHG
        # Negative - right turn | Positive - left turn
        # Get tilt from GYRO - no more than 15 degree bank to any side
        # https://www.grc.nasa.gov/www/k-12/airplane/turns.html
        CHG_HDG = CURR_HDG - REQ_HDG

        if abs(REQ_HDG) > 15 :
            # banking turn
            if REQ_HDG < 0: # LEFT BANK
                # bank 10° to the left, monitor bank angle from gyro
                BANK = 10
                AUTO_BANK = -1 * ((BANK - ACT_BANK)/10)
            if REQ_HDG > 0: # RIGHT BANK
                BANK = 10
                AUTO_BANK = (BANK - ACT_BANK)/10
        if abs(REQ_HDG) < 15 :
            # rudder turn
            #

        if abs(REQ_HDG) < 15 :
            # rudder turn

            # 180 turn is max aggro - multiplier 1
            # 1 dungaree is minimum - multiplier (1/180).
