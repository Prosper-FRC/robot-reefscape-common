// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.elevator.ElevatorConstants.MagneticSensorHardware;

/** 
 * A class to interact with the REV Magnetic sensor
 * 
 * Related documentation:
 * https://docs.revrobotics.com/rev-crossover-products/sensors/magnetic-limit-switch/specs
 */
public class MagneticSensorIORev implements MagneticSensorIO {
  private final DigitalInput kSensor;

  public MagneticSensorIORev(MagneticSensorHardware hardware) {
    kSensor = new DigitalInput(hardware.sensorChannel());
  }

  public void updateInputs(MagneticSensorIOInputs inputs) {
    // Assume sensor is always connected, could implement method for checking if its
    // actually still connected later
    inputs.isConnected = true;

    inputs.isActivated = kSensor.get();
  }
}
