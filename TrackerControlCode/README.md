# Tracker control software #

A few notes about the python files included here.

A redis server was used to coordinate the line for tracker updates and
for getting data off the line for reporting. Each time we try to
communicate with the tracker, we hold a lock on the line using
redislock.

## updateTrackerPos.py ##

This python file is called every 5 minutes from a cron job, each 5
minutes it uses the lat,long azimuth of the array and the current time
to determine the optimal angle of the array based on the sun
position. At the end of the day it starts backtracking.

Before each call, it reads sensors centrally located for the entire
array for both snow and wind.

If we detect snow, we physically write out a file to the computer - if
we have a power out situation, the file is persistent and will not
return to normal operations until it is manually cleared (i.e. we are
sure that all snow is off the panels)

If wind, it tabletops the array

Each tracker controller has a (typically) 5 degree hysterisis - so
there must be a more than 5 degree change in setpoint before the
tracker will try to move again.

Each tracker has 2 angle sensors on the front and back and typically
will average the angle between these two sensors. This was to ensure
that there was no slippage or difference between the front panel angle
and the back panel angle (we had problems at one point with this
happening)

Each tracker has a max amp rating and a 'soft' amp rating, if at any
time the amps from the motor go above the high amp rating it will flag
an error and if it goes above the soft rating for a time period it
will flag an error - resetTracker.py has been used to reset these
problems.

Possible error conditions are as follows:

#define STRAIN_ERROR 1  = Amerage above 'soft' level for a preset time period
#define FAST_BLOW_ERROR 2 = Amperage above 'high' amp level
#define EAST_LIMIT_ERROR 3  = east limit switch activated
#define WEST_LIMIT_ERROR 4 = west limit switch activated
#define STALL_ERROR 5 = too long a time of no panel movement when the motor has been engaged
#define ANGLE_DIFF_ERROR 6 = too much difference between front and back panel
#define EAST_ANGLE_EXCEED_ERROR 7 = Angle too high in the east direction (software defined failsafe for limit switch) 
#define WEST_ANGLE_EXCEED_ERROR 8 = Angle too high in the west direction (software defined failsafe for limit switch) 
#define EMERGENCY_STOP_ERROR 9 = e stop activated at console
