// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.elevator.ElevatorConstants.SensorHardware;

/** 
 * A class to interact with the REV Magnetic sensor
 * 
 * Related documentation:
 * https://docs.revrobotics.com/rev-crossover-products/sensors/magnetic-limit-switch/specs
 */
public class SensorIOMagnetic implements SensorIO {
  private final DigitalInput kSensor;

  public SensorIOMagnetic(SensorHardware hardware) {
    kSensor = new DigitalInput(hardware.sensorChannel());
  }
}
