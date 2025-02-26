// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/** Sensor interface for a DutyCycle encoder */
public interface DutyCycleEncoderIO {
  @AutoLog
  public static class DutyCycleEncoderIOInputs {
    public boolean isConnected = false;

    /** The duty cycle's signal frequency */
    public int frequencyHz = 0;
    public double dutyCycleReading = 0.0;
  }

  /**
   * Write data from the hardware to the inputs object
   * 
   * @param inputs The inputs object
   */
  public default void updateInputs(DutyCycleEncoderIOInputs inputs) {}
}
