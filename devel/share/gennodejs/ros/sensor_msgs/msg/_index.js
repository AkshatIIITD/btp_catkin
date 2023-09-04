
"use strict";

let CompressedImage = require('./CompressedImage.js');
let RegionOfInterest = require('./RegionOfInterest.js');
let PointCloud = require('./PointCloud.js');
let JoyFeedbackArray = require('./JoyFeedbackArray.js');
let PointCloud2 = require('./PointCloud2.js');
let RelativeHumidity = require('./RelativeHumidity.js');
let TimeReference = require('./TimeReference.js');
let Temperature = require('./Temperature.js');
let Imu = require('./Imu.js');
let FluidPressure = require('./FluidPressure.js');
let BatteryState = require('./BatteryState.js');
let MagneticField = require('./MagneticField.js');
let LaserScan = require('./LaserScan.js');
let JointState = require('./JointState.js');
let PointField = require('./PointField.js');
let NavSatFix = require('./NavSatFix.js');
let Joy = require('./Joy.js');
let Illuminance = require('./Illuminance.js');
let MultiEchoLaserScan = require('./MultiEchoLaserScan.js');
let NavSatStatus = require('./NavSatStatus.js');
let LaserEcho = require('./LaserEcho.js');
let CameraInfo = require('./CameraInfo.js');
let MultiDOFJointState = require('./MultiDOFJointState.js');
let Image = require('./Image.js');
let JoyFeedback = require('./JoyFeedback.js');
let ChannelFloat32 = require('./ChannelFloat32.js');
let Range = require('./Range.js');

module.exports = {
  CompressedImage: CompressedImage,
  RegionOfInterest: RegionOfInterest,
  PointCloud: PointCloud,
  JoyFeedbackArray: JoyFeedbackArray,
  PointCloud2: PointCloud2,
  RelativeHumidity: RelativeHumidity,
  TimeReference: TimeReference,
  Temperature: Temperature,
  Imu: Imu,
  FluidPressure: FluidPressure,
  BatteryState: BatteryState,
  MagneticField: MagneticField,
  LaserScan: LaserScan,
  JointState: JointState,
  PointField: PointField,
  NavSatFix: NavSatFix,
  Joy: Joy,
  Illuminance: Illuminance,
  MultiEchoLaserScan: MultiEchoLaserScan,
  NavSatStatus: NavSatStatus,
  LaserEcho: LaserEcho,
  CameraInfo: CameraInfo,
  MultiDOFJointState: MultiDOFJointState,
  Image: Image,
  JoyFeedback: JoyFeedback,
  ChannelFloat32: ChannelFloat32,
  Range: Range,
};
