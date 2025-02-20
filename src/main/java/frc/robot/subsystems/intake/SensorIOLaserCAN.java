// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import frc.robot.subsystems.intake.IntakeConstants.SensorConfiguration;
import au.grapplerobotics.ConfigurationFailedException;

/** 
 * A class to interact with the LaserCAN sensor 
 * 
 * Related documentation:
 * https://grapplerobotics.au/product/lasercan/
 */
public class SensorIOLaserCAN implements SensorIO {
  private final LaserCan kSensor;

  private final double detectionDistanceThresholdMilimeters;

  public SensorIOLaserCAN(SensorConfiguration configuration) {
    kSensor = new LaserCan(configuration.sensorId());
    detectionDistanceThresholdMilimeters = configuration.detectionThresholdMilimeters();
    try {  
      /*
       * See this comment and documentation about the units if x, y, w, h 
       * https://github.com/Prosper-FRC/robot-reefscape-common/pull/20#discussion_r1947983003
       * https://grapplerobotics.au/product/lasercan/
       */
      kSensor.setRangingMode(LaserCan.RangingMode.SHORT);
      kSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(
        configuration.x(),
        configuration.y(),
        configuration.w(),
        configuration.h()));
    } catch (ConfigurationFailedException e) {
      System.out.println("ERROR: Intake/SensorIOLaserCAN - Configuration failed, see stacktrace:\n" + e);
    }
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    // Call sensor measruement at start of loop, do not call multiple times as this could make the call
    // slower. See this issue comment for more info:
    // https://github.com/Prosper-FRC/robot-reefscape-common/pull/20#discussion_r1947927206
    Measurement sensorMeasurement = kSensor.getMeasurement();

    inputs.isConnected = sensorMeasurement.status == LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT;

    if (sensorMeasurement != null && inputs.isConnected) {
      inputs.detectsObject = sensorMeasurement.distance_mm < detectionDistanceThresholdMilimeters;
    } else {
      inputs.detectsObject = false;
    }
  }
}
