// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Sensor inteface for a CANrange sensor */
public interface SensorIO {
  @AutoLog
  public static class SensorIOInputs {
    public boolean isConnected = false;

    public boolean detectsObject = false;
  }

  /**
   * Write data from the hardware to the inputs object
   * 
   * @param inputs The inputs object
   */
  public default void updateInputs(SensorIOInputs inputs) {}
}
