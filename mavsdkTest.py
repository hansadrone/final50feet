#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This is a test script to demonstrate the usage of the hansadrone.com precision navigation service with a mavlink enabled drone or simulator
# If running this script in a Simulator, the drone takeoff location needs to be set to near local position (within "range") before running this script!
# Also, comment out the ultrasonic disctance sensor section if no sensor is available
# Set the system settings in the main function before running this script!

# Copyright 2022 hansadrone.com

# LICENSE STATEMENT
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
# associated documentation files (the "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is furnished to do so, subject
# to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies or substantial
# portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import asyncio
import requests
import pyproj
from math import radians, acos, sin, cos, tan, sqrt, atan2, pi, asin
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw)

# helper function to get drone telemetry
async def getPos(drone):
    # Get GPS position data from telemetry
    async for pos in drone.telemetry.position():
        lat, lon, alt = pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m
        return lat, lon, alt

# main function starts here ------------------------------------------------------------------------------------
async def run():

    # Hansadrone.com API key. use lowercase characters only
    APIkey = "<PUT YOUR API KEY HERE>"

    # system settings
    # altitude above ground for cruise flight (m)
    alt_cruise = 30.00
    # max speed during cruise (m/s)
    max_speed=2.00
    # speed for approach flight (m/s)
    apr_speed = 1.00
    # max distance between takeoff location and fixpoint (m). Also used for geofence range
    range=200.0
    # the maximum altitude above takeoff altitude (m) for geofence altitude
    max_alt=100.0
    # This is the maximum range of the distance sensor(cm)
    sensor_range=700

    # connect to drone
    drone = System()
    # connect to local jmavsim simulator on Windows or Linux PC
    # await drone.connect(system_address="udp://:14540")
    # connect to real drone over COM port on Windows PC
    await drone.connect(system_address="serial:///COM5")
    # connect to real drone over COM port on Linux PC
    # await drone.connect(system_address="serial:///path/to/serial/dev[:baudrate]")
    print("-- Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Drone discovered")
            break

    # update backend status to connected
    print("-- Setting drone status to connected")
    URL = "https://hansadrone.com/api/setDroneStatus?dStatus=connected&apiKey=%s"%(APIkey)
    r=requests.get(url=URL)

    # wait for drone to get GPS fix and obtain takeoff location
    print("-- Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    # get drone takeoff position
    async for pos in drone.telemetry.position():
        x, y, z = pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m
        print("Drone takeoff location\nLatitude: %s\nLongitude: %s\nAltitude: %.2fm"%(x, y, z))
        break

    # get battery status
    async for bat in drone.telemetry.battery():
      voltage, level = bat.voltage_v, bat.remaining_percent
      print("Battery capacity remaining: %.2f, Battery voltage: %.2fV"%(level, voltage))
      break

    # setting drone failsafe parameters
    print("-- setting drone failsafe parameters")
    # fly home if bat low or land if bat critical
    await drone.param.set_param_int('COM_LOW_BAT_ACT', 3)
    # return home if offboard signal lost
    await drone.param.set_param_int('COM_OBL_ACT', 2)
    # return home if RC connected but offboard signal lost
    await drone.param.set_param_int('COM_OBL_RC_ACT', 3)
    # time-out for offboard signal lost action
    await drone.param.set_param_float('COM_OF_LOSS_T', 1.0)
    # auto disarm after landing
    await drone.param.set_param_float('COM_DISARM_LAND', 10.0)
    # Return home at this altitude
    await drone.param.set_param_float('RTL_RETURN_ALT', alt_cruise)
    # time-out for GPS signal lost ACTION
    await drone.param.set_param_float('COM_POS_FS_DELAY', 1)
    # todo: deactivate loiter on GPS lost during approach fligh in GPS denied areas
    # await drone.param.set_param_int('NAV_GPS_LT', 0)

    # setting drone flight and mode parameters
    print("-- setting min takeoff altitude to %sm"%(alt_cruise))
    await drone.action.set_takeoff_altitude(alt_cruise)
    print("-- setting return to home altitude to %sm"%(alt_cruise))
    await drone.action.set_return_to_launch_altitude(alt_cruise)
    print("-- setting max speed to %sm/s"%(max_speed))
    await drone.action.set_maximum_speed(max_speed)
    print("-- setting RC mode to not required")
    await drone.param.set_param_int('COM_RC_IN_MODE', 1)

    # setting geofence parameters
    print("-- setting geofence around takeoff location with" + str(range) + "m radius and " + str(max_alt)+ "m altitude")
    await drone.param.set_param_float('GF_MAX_HOR_DIST', range)
    await drone.param.set_param_float('LNDMC_ALT_MAX', max_alt)

    # while loop to wait for mobile to connect. exit if not connected within 60 s
    print("-- waiting 60s for mobile to connect...")
    URL = "https://hansadrone.com/api/getMobileStatus/?apiKey=%s"%(APIkey)
    i=0
    while i<12 :
      r=requests.get(url=URL)
      data = r.json()
      MSTATUS=data['status']
      if MSTATUS=="connected" :
        print("Mobile Status: %s"%(MSTATUS))
        i=12
      elif i==11 :
        print("connection timed out, aborting")
        URL = "https://hansadrone.com/api/setDroneStatus?dStatus=unknown&apiKey=%s"%(APIkey)
        r=requests.get(url=URL)
        URL = "https://hansadrone.com/api/setMobileStatus?mStatus=unknown&apiKey=%s"%(APIkey)
        r=requests.get(url=URL)
        exit()
      else :
        await asyncio.sleep(5)
        i+=1

    # update backend status to available, this triggers UI flow on the smartphone app to define a corridor
    print("-- Setting drone status to available")
    URL = "https://hansadrone.com/api/setDroneStatus?dStatus=available&apiKey=%s"%(APIkey)
    r=requests.get(url=URL)

    # while loop to wait for the customer to define a corridor. exit if not defined within 60 s
    print("-- waiting 60s for mobile to define corridor...")
    URL = "https://hansadrone.com/api/getMobileStatus/?apiKey=%s"%(APIkey)
    i=0
    while i<12 :
      r=requests.get(url=URL)
      data = r.json()
      MSTATUS=data['status']
      if MSTATUS=="defined" :
        print("Mobile Status: %s"%(MSTATUS))
        i=12
      elif i==11 :
        print("connection timed out, aborting")
        URL = "https://hansadrone.com/api/setDroneStatus?dStatus=unknown&apiKey=%s"%(APIkey)
        r=requests.get(url=URL)
        URL = "https://hansadrone.com/api/setMobileStatus?mStatus=unknown&apiKey=%s"%(APIkey)
        r=requests.get(url=URL)
        exit()
      else :
        await asyncio.sleep(5)
        i+=1

    # wait for smartphone app and backend to sync the corridor data
    await asyncio.sleep(5)

    # get corridor data and check if not null
    URL = "https://hansadrone.com/api/getCorData/?apiKey=%s"%(APIkey)
    r=requests.get(url=URL)
    data = r.json()
    COMP=data['comp']
    INC=data['inc']
    LAT=data['lat']
    LONG=data['long']
    alt_user=data['alt']
    print("-- Getting corridor data\nCompass: %s\nInclination: %s\nLatitude: %s\nLongitude: %s"%(COMP, INC, LAT, LONG))
    if COMP==0 or INC==0 or LAT==0 or LONG==0:
        print("Corridor data not valid, aborting")
        URL = "https://hansadrone.com/api/setDroneStatus?dStatus=unknown&apiKey=%s"%(APIkey)
        r=requests.get(url=URL)
        URL = "https://hansadrone.com/api/setMobileStatus?mStatus=unknown&apiKey=%s"%(APIkey)
        r=requests.get(url=URL)
        exit()

    # get fixpoint data based on cruise altitude
    URL = "https://hansadrone.com/api/getFixPoint?height=%s&apiKey=%s"%(alt_cruise-alt_user, APIkey)
    r=requests.get(url=URL)
    data = r.json()
    LAT=data['fixLat']
    LONG=data['fixLong']
    print("-- Fixpoint position: \nLatitude: %s\nLongitude: %s"%( LAT, LONG))

    # check if fix point is within range from takeoff location
    print("-- Calculating distance between takeoff location and fixpoint")
    R = 6373.0
    lat1 = float(LAT)
    lon1 = float(LONG)
    inc = float(INC)*pi/180
    dlon = (y - lon1)*pi/180
    dlat = (x - lat1)*pi/180
    a = (sin(dlat/2))**2 + cos(lat1) * cos(x) * (sin(dlon/2))**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    distance= R * c * 1000
    print("Distance to fixpoint: %.2fm"%distance)
    if distance > range:
      print("Fixpoint too far away from takeoff location, aborting")
      URL = "https://hansadrone.com/api/setDroneStatus?dStatus=unknown&apiKey=%s"%(APIkey)
      r=requests.get(url=URL)
      URL = "https://hansadrone.com/api/setMobileStatus?mStatus=unknown&apiKey=%s"%(APIkey)
      r=requests.get(url=URL)
      exit()

    # calculate bearing from takeoff location to fixpint
    geodesic = pyproj.Geod(ellps='WGS84')
    fwd_azimuth, back_azimuth, distance = geodesic.inv(x, y, lat1, lon1)
    # coordinates system of the pyproj is facing east with neagitiv angles south
    if fwd_azimuth>=0 and fwd_azimuth<=180:
      fwd_azimuth=360-fwd_azimuth
    elif fwd_azimuth <0:
      fwd_azimuth=fwd_azimuth *-1
    fwd_azimuth=fwd_azimuth+90
    if fwd_azimuth>360:
      fwd_azimuth=fwd_azimuth-360
    print("-- Bearing from takeoff position to fixpoint: %s" %fwd_azimuth)

    # update backend status to underway
    print("-- Setting drone status to underway")
    URL = "https://hansadrone.com/api/setDroneStatus?dStatus=underway&apiKey=%s"%(APIkey)
    r=requests.get(url=URL)

    # drone arming
    print("-- Arming")
    await drone.action.arm()
    await asyncio.sleep(5)

    # takeoff and monitor altitude while taking off
    print("-- Taking off")
    await drone.action.takeoff()
    alt=0.00
    while alt < alt_cruise:
      async for pos in drone.telemetry.position():
        alt=pos.relative_altitude_m
        alt=alt+1.00
        print("altitude: %.2fm" % alt)
        break
      await asyncio.sleep(1)
    await asyncio.sleep(2)

    # fly to fixpoint and monitor distance
    print("-- flying to fixpoint")
    # altitude1 is absolute height above NN
    altitude1=alt_cruise+z
    await drone.action.goto_location(lat1, lon1, altitude1, fwd_azimuth)
    distance=20.00
    while distance > 1.0:
      async for pos in drone.telemetry.position():
        lat, lon, alt=pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m
        print("lat: %s, lon: %s, alt: %.2fm"%(lat, lon, alt))
        slat=radians(lat)
        slon=radians(lon)
        elat=radians(lat1)
        elon=radians(lon1)
        distance=6371010 * acos(sin(slat)*sin(elat) + cos(slat)*cos(elat)*cos(slon-elon))
        print("distance to fixpoint: %.2fm"%(distance))
        break
      await asyncio.sleep(2)
    await asyncio.sleep(5)

    # rotate drone towards customer
    print("-- rotating towards user")
    comp= float(COMP)
    if comp <=180:
      comp1=comp+180
    else:
      comp1=comp-180
    await drone.action.goto_location(lat1, lon1, altitude1, comp1)
    await asyncio.sleep(2)


    # update backend status to atfixpoint. this will activate drone recognition on the smartphone app
    # turn on drone lights here
    print("-- Setting drone status to atfixpoint")
    URL = "https://hansadrone.com/api/setDroneStatus?dStatus=atfixpoint&apiKey=%s"%(APIkey)
    r=requests.get(url=URL)
    await asyncio.sleep(3)

    # while loop to wait for drone to be recognized. exit if not recognized within 60 s
    print("-- waiting 60s for drone to be recognized...")
    URL = "https://hansadrone.com/api/getMobileStatus/?apiKey=%s"%(APIkey)
    i=0
    while i<12 :
      r=requests.get(url=URL)
      data = r.json()
      MSTATUS=data['status']
      if MSTATUS=="recognized" :
        print("Mobile Status: %s"%(MSTATUS))
        i=12
      elif i==11 :
        print("connection timed out, flying back home")
        URL = "https://hansadrone.com/api/setDroneStatus?dStatus=unknown&apiKey=%s"%(APIkey)
        r=requests.get(url=URL)
        URL = "https://hansadrone.com/api/setMobileStatus?mStatus=unknown&apiKey=%s"%(APIkey)
        r=requests.get(url=URL)
        print("-- returning home")
        await drone.action.return_to_launch()
        i=12
        exit()
      else :
        await asyncio.sleep(5)
        i+=1

    # offboard mode defaults
    s_distance=1000;
    north=0.0
    east=0.0
    altitude2=-alt_cruise
    comp1RAD = comp1 *pi/180
    print("-- setting min takeoff altitude to 0m for approach")
    await drone.action.set_takeoff_altitude(0.00)

    # get deviations from server
    URL = "https://hansadrone.com/api/getDeviations/?apiKey=%s"%(APIkey)
    r=requests.get(url=URL)
    data = r.json()
    devX=float(data['devX'])
    devY=float(data['devY'])
    angX=devX * pi / 180
    angY=devY * pi /180
    print("-- drone recognized. devX: %s devY: %s"%(devX, devY))

    # get actual altitude from drone telemetry.
    lat, lon, alt = await getPos(drone)
    print("Drone altitude: " + str(alt))

    # calculate corrected yaw angle for the drone during approach flight. This is in degrees.
    angle = 0.5 * angX
    yaw=(comp1RAD - angX) * 180 / pi
    if yaw > 360:
      yaw = yaw-360
    if yaw <0:
      yaw=yaw+360
    print("Corrected yaw angle: %s"%yaw)

    # calculate distance based on recognized inclination angle and altitude difference
    # distance is the absolute distance including height difference, not the distance on ground.
    dist = (alt-alt_user) /sin(inc+angY)

    # calculate absolute deviations
    # absolute deviations are relative to a corridor centric coordinate system and not to the ground based system
    absX = dist * sin(angX)
    absY= dist*sin(angY)
    print("distance: " + str(dist)+ "distance1: " + " altitude: " + str(alt) + "absX: " + str(absX) + " absY: " + str(absY))

    # start offboard mode
    # set first setpoint in velocityNED before offboard mode can be enabled
    # repeat again as this sometimes doesnt get enabled
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.00, 0.00, 0.00, yaw))
    await asyncio.sleep(1)
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.00, 0.00, 0.00, yaw))
    await asyncio.sleep(1)

    print("-- starting offboard mode")
    try:
      await drone.offboard.start()
    except OffboardError as error:
      print(f"-- Starting offboard mode failed with error code: {error._result.result}")
      print("-- returning home")
      await drone.action.return_to_launch()
      URL = "https://hansadrone.com/api/setDroneStatus?dStatus=unknown&apiKey=%s"%(APIkey)
      r=requests.get(url=URL)
      URL = "https://hansadrone.com/api/setMobileStatus?mStatus=unknown&apiKey=%s"%(APIkey)
      r=requests.get(url=URL)
      return

    # this part to be repeated for each step until drone has reached sensor contact
    # stop loop if distance < 765cm or <alt_user and start final approach mode
    # all angles in RAD in this section

    while s_distance>=sensor_range and alt >alt_user:

      # while loop to wait for drone to be recognized. exit if not recognized within 10 s
      print("-- waiting 10s for drone to be recognized...")
      URL = "https://hansadrone.com/api/getMobileStatus/?apiKey=%s"%(APIkey)
      i=0
      while i<11 :
        r=requests.get(url=URL)
        data = r.json()
        MSTATUS=data['status']
        if MSTATUS=="recognized" :
          print("Mobile Status: %s"%(MSTATUS))
          i=11
        elif i==10 :
          print("-- returning home")
          await drone.action.return_to_launch()
          i=11
          print("connection timed out, flying back home")
          URL = "https://hansadrone.com/api/setDroneStatus?dStatus=unknown&apiKey=%s"%(APIkey)
          r=requests.get(url=URL)
          URL = "https://hansadrone.com/api/setMobileStatus?mStatus=unknown&apiKey=%s"%(APIkey)
          r=requests.get(url=URL)
          exit()
        else :
          await asyncio.sleep(1)
          i+=1

      # get deviations from server
      URL = "https://hansadrone.com/api/getDeviations/?apiKey=%s"%(APIkey)
      r=requests.get(url=URL)
      data = r.json()
      devX=float(data['devX'])
      devY=float(data['devY'])
      if isEqual(devX, 99) or isEqual(devY, 99.0):
        print("-- drone outside corridor, trying again..")
        continue
      else:
        angX=devX * pi / 180
        angY=devY * pi /180
        print("-- drone recognized. devX: %s devY: %s"%(devX, devY))

      # get actual position from drone telemetry.
      lat, lon, alt = await getPos(drone)
      print("Drone altitude: " + str(alt))

      # calculate distance, stepLength and absolute deviations
      dist =  (alt-alt_user) /tan(inc+angY)
      stepLength=dist/3.0
      absX = dist * sin(angX)
      absY = alt - alt_user - tan(inc) * dist
      print("distance: " + str(dist)+ " altitude: " + str(alt) + "absX: " + str(absX) + " absY: " + str(absY))

      # calculate new x position: moving inwards absX at corrected angle, moving forwards at stepLength
      # todo: catch 0 and 360 degree issues and validate formula
      l=sqrt(absX**2 + stepLength**2)
      if angX < 0:
        stepAngle = comp1RAD -0.5*pi + angle + asin(stepLength/l)
      else:
        stepAngle = comp1RAD +0.5*pi -angle - asin(stepLength/l)
      if stepAngle > 2*pi:
        stepAngle = stepAngle -2*pi
      north = l * cos(stepAngle)
      east = l * sin(stepAngle)

      # calculate new Y position: move up/down by absY, forward a step and down according to inc
      down = -(alt - stepLength * tan(inc) -absY)
      if down < -alt_user:
        down = -alt_user
      print("new drone position \nNorth: " + str(north) + "\nEast: " + str(east) + "\nDown: " + str(down) + "\nYaw: " + str(yaw))

      # convert NED coordinates to NED velocity
      # calculate absolute distance to next setpoint
      s = sqrt(north**2 + east**2 + down**2)
      # calculate the time it takes to get there
      t = s / apr_speed
      # calculate velocity components, down velocity is positive
      v_north = north / t
      v_east = east / t
      v_down = (alt+down) / t

      # Start offboard mode NEDvelocity for time t and then stop again
      print("-- Starting offboard velocity_NED. V_north:" + str(v_north) + " v_east: "+ str(v_east) +" v_down: " + str(v_down) + " time: " + str(t))
      await drone.offboard.set_velocity_ned(VelocityNedYaw(v_north, v_east, v_down, yaw))
      await asyncio.sleep(t)
      await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, yaw))
      await asyncio.sleep(1)

      # get sensor distance
      # comment out this section  when flying in a simulator
      async for sensor in drone.telemetry.distance_sensor():
        s_distance=sensor.current_distance_m
        print("sensor distance: %s"%(s_distance))
        break
      await asyncio.sleep(1)

      # get new actual position from drone telemetry.
      lat, lon, alt = await getPos(drone)

    # end of while loop

    # final approach. user can turn his smartphone away now, open hatch and turn off the light
    print("-- Starting final approach")

    # get actual position from drone telemetry.
    lat, lon, alt = await getPos(drone)
    print("Drone altitude: " + str(alt))

    # get deviations from server
    URL = "https://hansadrone.com/api/getDeviations/?apiKey=%s"%(APIkey)
    r=requests.get(url=URL)
    data = r.json()
    devX=float(data['devX'])
    devY=float(data['devY'])
    angX=devX * pi / 180
    angY=devY * pi /180

    # comment out this section and set sensor distance to fix value if flying in a simulator
    async for sensor in drone.telemetry.distance_sensor():
      s_distance=sensor.current_distance_m
      s_distance=500
      print("sensor distance: %s"%(s_distance))
      break

    # update backend status to "arrived" to trigger the UI in the app to put the smartphone aside
    print("-- Setting drone status to arrived")
    URL = "https://hansadrone.com/api/setDroneStatus?dStatus=completed&apiKey=%s"%(APIkey)
    r=requests.get(url=URL)
    await asyncio.sleep(2)

    # calculate final step
    distance=s_distance/100
    absX=distance*sin(angX)
    absY=distance * sin(angY)

    # calculate final x position: moving inwards at recognized angle, forward at distance
    # todo: catch 0 and 360 degree issues and validate formula
    if angX < 0:
      stepAngle = comp1RAD -angX
    else:
      stepAngle = comp1RAD +angX
    if stepAngle > 2*pi:
      stepAngle = stepAngle -2*pi
    north = distance * cos(stepAngle)
    east = distance * sin(stepAngle)

    # calculate new Y position: move up/down by absY, forward at distance and down according to inc
    down = -(alt - stepLength * tan(inc) -absY)
    if down < -alt_user:
      down = -alt_user
    print("new drone position \nNorth: " + str(north) + "\nEast: " + str(east) + "\nDown: " + str(down) + "\nYaw: " + str(yaw))

    # convert NED coordinates to NED velocity
    # calculate absolute distance to the customer
    s = sqrt(north**2 + east**2 + down**2)
    # calculate the time it takes to get there
    t = s / apr_speed
    # calculate velocity components, down velocity is positive
    v_north = north / t
    v_east = east / t
    v_down = (alt+down) / t

    # start NEDvelocity and monitor sensor disctance
    await drone.offboard.set_velocity_ned(VelocityNedYaw(v_north, v_east, v_down, yaw))
    while s_distance > 100:
      async for sensor in drone.telemetry.distance_sensor():
        s_distance=sensor.current_distance_m
        print("sensor distance: %s"%(s_distance))
        break
      await asyncio.sleep(1)

    # stop in front of customer for unloading
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, yaw))
    await asyncio.sleep(10)

    # stop offboard mode. this requires GPS fix
    print("-- stopping offboard mode")
    try:
      await drone.offboard.stop()
    except OffboardError as error:
      print(f"Stopping offboard mode failed with error code: {error._result.result}")
      print("returning home")
      await drone.action.return_to_launch()
      URL = "https://hansadrone.com/api/setDroneStatus?dStatus=unknown&apiKey=%s"%(APIkey)
      r=requests.get(url=URL)
      URL = "https://hansadrone.com/api/setMobileStatus?mStatus=unknown&apiKey=%s"%(APIkey)
      r=requests.get(url=URL)
      return

    # update backend status to completed
    print("-- Setting drone status to completed")
    URL = "https://hansadrone.com/api/setDroneStatus?dStatus=completed&apiKey=%s"%(APIkey)
    r=requests.get(url=URL)
    await asyncio.sleep(2)

    # drone flying back to fixpoint
    print("-- setting takeoff altitude to %.2fm"%(alt_cruise))
    await drone.action.set_takeoff_altitude(alt_cruise)
    print("-- returning to fixpoint")
    # todo: this might fail in case no GPS signal. better use offboard mode to fly back to fixpoint
    await drone.action.goto_location(lat1, lon1, altitude1, comp)
    distance=20.00
    while distance > 2.0:
      async for pos in drone.telemetry.position():
        lat, lon, alt=pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m
        print("lat: %s, lon: %s, alt: %.2fm"%(lat, lon, alt))
        slat=radians(lat)
        slon=radians(lon)
        elat=radians(lat1)
        elon=radians(lon1)
        distance=6371010 * acos(sin(slat)*sin(elat) + cos(slat)*cos(elat)*cos(slon-elon))
        print("distance to fixpoint: %.2fm"%(distance))
        break
      await asyncio.sleep(2)
    await asyncio.sleep(2)

    # fly back to takeoff position
    print("-- flying to takeoff position")
    # altitude1 is absolute height above NN
    altitude1=alt_cruise+z
    await drone.action.goto_location(x, y, altitude1, back_azimuth)
    distance=20.00
    while distance > 1.0:
      async for pos in drone.telemetry.position():
        lat, lon, alt=pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m
        print("lat: %s, lon: %s, alt: %.2fm"%(lat, lon, alt))
        slat=radians(lat)
        slon=radians(lon)
        elat=radians(x)
        elon=radians(y)
        distance=6371010 * acos(sin(slat)*sin(elat) + cos(slat)*cos(elat)*cos(slon-elon))
        print("distance to takeoff position: %.2fm"%(distance))
        break
      await asyncio.sleep(2)
    await asyncio.sleep(2)

    # land and monitor altitude while landing
    print("-- Landing")
    await drone.action.land()
    alt=alt_cruise
    while alt > 1.0:
      async for pos in drone.telemetry.position():
        lat, lon, alt=pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m
        print("Latitude: %s, Longitude: %s, Altitude: %.2fm"%(lat, lon, alt))
        break
      async for sensor in drone.telemetry.distance_sensor():
        s_distance=sensor.current_distance_m
        print("sensor distance: %scm"%(s_distance))
        break
      await asyncio.sleep(1)
    # end of while

    print("-- Setting drone status to unknown")
    URL = "https://hansadrone.com/api/setDroneStatus?dStatus=unknown&apiKey=%s"%(APIkey)
    r=requests.get(url=URL)
    URL = "https://hansadrone.com/api/setMobileStatus?mStatus=unknown&apiKey=%s"%(APIkey)
    r=requests.get(url=URL)

    await asyncio.sleep(10)
    print("-- disarming")
    await drone.action.disarm()
    print("-- Mission completed!")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
