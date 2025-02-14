// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final ClimbIO[] kHardware;
  private final ClimbIOInputsAutoLogged[] kInputs;

  /** Creates a new Climb. */
  public Climb(ClimbIO... io) {
    kHardware = new ClimbIO[io.length];
    kInputs = new ClimbIOInputsAutoLogged[io.length];

    for (int i = 0; i < kHardware.length; i++) {
      kHardware[i] = io[i];
      kInputs[i] = new ClimbIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
