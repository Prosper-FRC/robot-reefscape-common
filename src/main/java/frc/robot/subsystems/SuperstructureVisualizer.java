// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.intake.Intake.Gamepiece;

/** Class for 3D visualization of the superstructure */
public class SuperstructureVisualizer {
  private final Pose3d kHoldingCoralPose = new Pose3d(
    -0.03,
    0.21,
    0.65,
    new Rotation3d(0.0, Rotation2d.fromDegrees(-32.0).getRadians(), 0.0)
  );

  private final Pose3d kHoldingAlgaePose = new Pose3d(
    -0.15,
    0.23,
    0.95,
    new Rotation3d(0.0, 0.0, 0.0)
  );

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
    Logger.recordOutput("Superstructure/Poses", new Pose3d[] {
      new Pose3d(),
      new Pose3d(),
      new Pose3d(),
      new Pose3d()
    });

    Logger.recordOutput("Superstructure/CoralPose", new Pose3d());
    Logger.recordOutput("Superstructure/AlgaePose", new Pose3d());
  }

  public void updateVisualizer(double elevatorPositionMeters, Rotation2d pivotPosition, Rotation2d climbPosition) {
    Logger.recordOutput("Superstructure/Poses", new Pose3d[] {
      updateMiddleStage(elevatorPositionMeters),
      updateInnerStage(elevatorPositionMeters),
      updatePivot(elevatorPositionMeters, pivotPosition),
      updateClimb(climbPosition)
    });
  }

  public void updateGamepiece(
    boolean hasGamepiece, 
    Gamepiece gamepiece, 
    Pose2d robotPose, 
    double elevatorPositionMeters, 
    Rotation2d pivotPosition) {
    if (!hasGamepiece) {
      // If we do not have a gamepiece, set both poses back to 0
      Logger.recordOutput("Superstructure/CoralPose", kHoldingCoralPose);
      Logger.recordOutput("Superstructure/AlgaePose", kHoldingAlgaePose);
    } else {
      if (gamepiece == Gamepiece.kCoral) {
        // Reset algae pose
        Logger.recordOutput("Superstructure/AlgaePose", kHoldingAlgaePose);

        Logger.recordOutput("Superstructure/CoralPose",
          new Pose3d(robotPose)
            .transformBy(
              new Transform3d(
                Pose3d.kZero, 
                new Pose3d(
                  kHoldingCoralPose.getX(),
                  kHoldingCoralPose.getY(),
                  kHoldingCoralPose.getZ() + elevatorPositionMeters,
                  kHoldingCoralPose.getRotation()
                )))
        );
      } else if (gamepiece == Gamepiece.kAlgae) {
        // Reset coral pose
        Logger.recordOutput("Superstructure/CoralPose", kHoldingCoralPose);

        /*
         * TODO Add this maybe later:
         * To get the open position of the picker, take the elevator position and
         * substract or add the sine of the pivot angle (ensure that there is an
         * offset for the pivot angle since 0 is straight up, so probs minus 90
         * degrees or something like that)
         */

        Logger.recordOutput("Superstructure/AlgaePose",
          new Pose3d(robotPose)
            .transformBy(
              new Transform3d(
                Pose3d.kZero, 
                new Pose3d(
                  kHoldingAlgaePose.getX(),
                  kHoldingAlgaePose.getY(),
                  kHoldingAlgaePose.getZ() + elevatorPositionMeters,
                  kHoldingAlgaePose.getRotation()
                )))
        );
      }
    }
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
