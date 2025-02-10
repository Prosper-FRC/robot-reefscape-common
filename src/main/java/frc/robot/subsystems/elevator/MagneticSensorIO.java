// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for a digital sensor - used to "home" the elevator */
public interface MagneticSensorIO {
  @AutoLog
  public static class MagneticSensorIOInputs {
    public boolean isConnected = false;

    /** 
     * Boolean to hold whether or not the digital input device has been "activated"
     * See these docs:
     * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/digital-inputs-software.html#digital-inputs-software
     */
    public boolean isActivated = false;
  }

  /**
   * Write data from the hardware to the inputs object
   * 
   * @param inputs The inputs object
   */
  public default void updateInputs(MagneticSensorIOInputs inputs) {}
}
