#!/usr/bin/env python

import cffirmware


ctrl = cffirmware.controllerRls_t()
cffirmware.controllerRlsInit(ctrl)

# ctrl = cffirmware.controllerMellinger_t()
# cffirmware.controllerMellingerInit(ctrl)


control = cffirmware.control_t()
setpoint = cffirmware.setpoint_t()
setpoint.mode.z = cffirmware.modeAbs
setpoint.position.z = 5.0
setpoint.mode.x = cffirmware.modeVelocity
setpoint.velocity.x = 0
setpoint.mode.y = cffirmware.modeVelocity
setpoint.velocity.y = 0
setpoint.mode.yaw = cffirmware.modeVelocity
setpoint.attitudeRate.yaw = 0

state = cffirmware.state_t()
state.attitude.roll = 0
state.attitude.pitch = -0 # WARNING: This needs to be negated
state.attitude.yaw = 0
state.position.x = 0
state.position.y = 0
state.position.z = 0
state.velocity.x = 0
state.velocity.y = 0
state.velocity.z = 0

sensors = cffirmware.sensorData_t()
sensors.gyro.x = 0
sensors.gyro.y = 0
sensors.gyro.z = 0

step = 100

cffirmware.controllerRls(ctrl, control, setpoint,sensors,state,step)
#cffirmware.controllerMellinger(ctrl, control, setpoint,sensors,state,step)



print("thrust: ", control.thrust)
