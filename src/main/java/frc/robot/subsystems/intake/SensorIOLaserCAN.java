// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Millimeters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import au.grapplerobotics.ConfigurationFailedException;

/** 
 * A class to interact with the CANRange sensor 
 * 
 * Related documentation:
 * https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/canrange/index.html
 * https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/CANrange.html
 */
public class SensorIOLaserCAN implements SensorIO {
  private final LaserCan kSensor = new LaserCan(IntakeConstants.kCANrangeID);
  

  private StatusSignal<Boolean> detectsObject;

  public SensorIOLaserCAN() {
    try
    {  
      kSensor.setRangingMode(LaserCan.RangingMode.SHORT);
      kSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8,8,16,16));
    }
    catch(ConfigurationFailedException e)
    {
      System.out.println("Configuration Failed! " +e);
    }


    // kRangeConfiguration.ProximityParams.ProximityThreshold = 
    //   IntakeConstants.kSensorConfiguration.kDetectionThresholdMeters();

    // kSensor.getConfigurator().apply(kRangeConfiguration);

    // detectsObject = kSensor.getIsDetected();

    // BaseStatusSignal.setUpdateFrequencyForAll(IntakeConstants.kStatusSignalUpdateFrequencyHz, 
    //   detectsObject);

    // // Optimize the CANBus utilization by explicitly telling all CAN signals we
    // // are not using to simply not be sent over the CANBus
    // kSensor.optimizeBusUtilization(0.0, 1.0);
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    // inputs.isConnected = BaseStatusSignal.refreshAll(detectsObject).isOK();
    inputs.isConnected = true;

    if (kSensor.getMeasurement() != null) {
      inputs.detectsObject = kSensor.getMeasurement().distance_mm < 5;
    } else {
      inputs.detectsObject = false;
    }
  }
}
