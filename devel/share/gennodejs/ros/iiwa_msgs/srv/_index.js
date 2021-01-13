
"use strict";

let SetPTPJointSpeedLimits = require('./SetPTPJointSpeedLimits.js')
let ConfigureControlMode = require('./ConfigureControlMode.js')
let SetPTPCartesianSpeedLimits = require('./SetPTPCartesianSpeedLimits.js')
let SetWorkpiece = require('./SetWorkpiece.js')
let SetSpeedOverride = require('./SetSpeedOverride.js')
let TimeToDestination = require('./TimeToDestination.js')
let SetEndpointFrame = require('./SetEndpointFrame.js')
let SetSmartServoLinSpeedLimits = require('./SetSmartServoLinSpeedLimits.js')
let SetSmartServoJointSpeedLimits = require('./SetSmartServoJointSpeedLimits.js')

module.exports = {
  SetPTPJointSpeedLimits: SetPTPJointSpeedLimits,
  ConfigureControlMode: ConfigureControlMode,
  SetPTPCartesianSpeedLimits: SetPTPCartesianSpeedLimits,
  SetWorkpiece: SetWorkpiece,
  SetSpeedOverride: SetSpeedOverride,
  TimeToDestination: TimeToDestination,
  SetEndpointFrame: SetEndpointFrame,
  SetSmartServoLinSpeedLimits: SetSmartServoLinSpeedLimits,
  SetSmartServoJointSpeedLimits: SetSmartServoJointSpeedLimits,
};
