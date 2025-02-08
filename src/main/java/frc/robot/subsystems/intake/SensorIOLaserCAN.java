// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.ConfigurationFailedException;

/** 
 * A class to interact with the LaserCAN sensor 
 * 
 * Related documentation:
 * https://grapplerobotics.au/product/lasercan/
 */
public class SensorIOLaserCAN implements SensorIO {
  private final LaserCan kSensor = new LaserCan(IntakeConstants.kCANrangeID);

  public SensorIOLaserCAN() {
    try {  
      kSensor.setRangingMode(LaserCan.RangingMode.SHORT);
      kSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8,8,16,16));
    } catch(ConfigurationFailedException e) {
      System.out.println("Configuration Failed! " +e);
    }
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.isConnected = kSensor.getMeasurement().status == LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT;

    if (kSensor.getMeasurement() != null && inputs.isConnected) {
      inputs.detectsObject = kSensor.getMeasurement().distance_mm < 5;
    } else {
      inputs.detectsObject = false;
    }
  }
}
