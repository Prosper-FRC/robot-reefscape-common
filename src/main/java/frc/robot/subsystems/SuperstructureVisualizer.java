// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Class for 3D visualization of the superstructure */
public class SuperstructureVisualizer {
  public enum Gamepiece {
    kNone,
    kCoral,
    kAlgae
  }

  private Gamepiece currentGamepiece = Gamepiece.kNone;

  private final Pose3d kMiddleStageOrigin = new Pose3d(
    0.0,
    0.065, // This is the axis that will move (vertical)
    0.07,
    new Rotation3d(0.0, 0.0, 0.0) // No need for rotations if already zeroed
  );
  private final Pose3d kInnerStageOrigin = new Pose3d(
    0.0,
    0.065, // This is the axis that will move (vertical)
    0.096,
    new Rotation3d(0.0, 0.0, 0.0)
  );
  private final Pose3d kPivotOrigin = new Pose3d(
    -0.165,
    0.215, // This is an axis that will move (vertical) because elevator
    0.428,
    new Rotation3d(
        0.0, 
        0.0, // This is an axis that will move (angularly) where negative is out
        0.0
    )
  );
  private final Pose3d kClimbOrigin = new Pose3d(
    -0.015,
    -0.32,
    0.44,
    new Rotation3d(
        0.0, // This is an axis that will move (angularly) where negative is out  
        0.0,
        0.0
    )
  );

  /** Create a new superstructure visualizer */
  public SuperstructureVisualizer() {
    Logger.recordOutput("SuperstructurePoses", new Pose3d[] {
      new Pose3d(),
      new Pose3d(),
      new Pose3d(),
      new Pose3d()
    });
  }

  public void updateVisualizer(double elevatorPositionMeters, Rotation2d pivotPosition, Rotation2d climbPosition) {
    Logger.recordOutput("SuperstructurePoses", new Pose3d[] {
      updateMiddleStage(elevatorPositionMeters),
      updateInnerStage(elevatorPositionMeters),
      updatePivot(elevatorPositionMeters, pivotPosition),
      updateClimb(climbPosition)
    });
  }

  private Pose3d updateMiddleStage(double elevatorPositionMeters) {
    return kMiddleStageOrigin.transformBy(
      new Transform3d(0.0, 0.0, elevatorPositionMeters / 2.0, new Rotation3d()));
  }

  private Pose3d updateInnerStage(double elevatorPositionMeters) {
    return kInnerStageOrigin.transformBy(
      new Transform3d(0.0, 0.0, elevatorPositionMeters, new Rotation3d()));
  }

  private Pose3d updatePivot(double elevatorPositionMeters, Rotation2d pivotPosition) {
    return kPivotOrigin.transformBy(
      new Transform3d(
        0.0, 
        0.0, 
        elevatorPositionMeters, 
        new Rotation3d(
          0.0,
          -pivotPosition.getRadians(),
          0.0
        )));
  }

  private Pose3d updateClimb(Rotation2d climbPosition) {
    return kClimbOrigin.transformBy(
      new Transform3d(
        0.0, 
        0.0, 
        0.0, 
        new Rotation3d(
          climbPosition.getRadians(),
          0.0,
          0.0
        )));    
  }
}
