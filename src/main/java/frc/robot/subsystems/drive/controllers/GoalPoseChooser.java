package frc.robot.subsystems.drive.controllers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.utils.math.AllianceFlipUtil;

/* Chooses pose based of strategy and psoe */ 
public class GoalPoseChooser {
    public static enum CHOOSER_STRATEGY {
        kTest,
        kReefHexagonal,
        kCustom,
        kIntake,
        kNet
    }

    public static enum SIDE {
        LEFT, RIGHT
    }

    private static Pose2d customGoal = FieldConstants.AL;
    private static SIDE side = SIDE.LEFT;

    public static Pose2d getGoalPose(CHOOSER_STRATEGY strategy, Pose2d pose) {
        switch(strategy) {
            case kTest:
                return new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(60));
            case kReefHexagonal:
                return getReefHexagonalPose(pose);
            case kCustom:
                return customGoal;
            case kIntake:
                return getIntakePose(pose);
            case kNet:
                return getNetPose(pose);
        }
        return new Pose2d();
    }

    /* Splits the field into hexagon regions of the reef 
     * We got the left or right side of the side we are closest
     */
    public static Pose2d getReefHexagonalPose(Pose2d robotPose) {
        Rotation2d angleFromReefCenter = turnFromReefOrigin(robotPose);
        Pose2d goal;
        if(inBetween(-30.0, 30.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "A");
            if(side.equals(SIDE.LEFT)) {
                goal = FieldConstants.DL;
            } else goal = FieldConstants.DR;
        } else if(inBetween(30.0, 90.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "B");
            if(side.equals(SIDE.LEFT)) {
                goal = FieldConstants.EL;
            } else goal = FieldConstants.ER;
        } else if(inBetween(90.0, 150.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "C");
            if(side.equals(SIDE.LEFT)) {
                goal = FieldConstants.FL;
            } else goal = FieldConstants.FR;
            // Skipped -150 to 150 because the inBetween function miscopes
            // Putting it in else covers the remainder of the hexagon scope
        } else if(inBetween(-150.0, -90.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "E");
            if(side.equals(SIDE.LEFT)) {
                goal = FieldConstants.BL;
            } else goal = FieldConstants.BR;
        } else if(inBetween(-90.0, -30.0, angleFromReefCenter.getDegrees())){
            Logger.recordOutput("Drive/ReefSide", "F");
            if(side.equals(SIDE.LEFT)) {
                goal = FieldConstants.CL;
            } else goal = FieldConstants.CR;
        } else {
            Logger.recordOutput("Drive/ReefSide", "D");
            if(side.equals(SIDE.LEFT)) {
                goal = FieldConstants.AL;
            } else goal = FieldConstants.AR;
        }
        Logger.recordOutput("Drive/SelectedSide", side);

        return AllianceFlipUtil.apply(goal);
    }

    public static Pose2d getIntakePose(Pose2d robotPose) {
        if(DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            return AllianceFlipUtil.apply((robotPose.getY() < Constants.kFieldWidthMeters / 2.0) ? FieldConstants.IR : FieldConstants.IL);
        } else return AllianceFlipUtil.apply((robotPose.getY() < Constants.kFieldWidthMeters / 2.0) ? FieldConstants.IL : FieldConstants.IR);
    }

    /* DO NOT USE X COORDINATE, REPLACE y holonomic speeds with driver controller when using this! */
    public static Pose2d getNetPose(Pose2d robotPose) {
        return AllianceFlipUtil.apply(new Pose2d(FieldConstants.kXNetLineMeters, 0.0, Rotation2d.kZero));
    }

    /* Accoumts for rotation from reef, and offsets for red-side logic */
    public static Rotation2d turnFromReefOrigin(Pose2d robotPose) {
        Pose2d reefCenter = AllianceFlipUtil.apply(FieldConstants.kReefCenter);
        Rotation2d angleFromReefCenter = Rotation2d.fromRadians(
            Math.atan2(
                robotPose.getY() - reefCenter.getY(), 
                robotPose.getX() - reefCenter.getX()));
        if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) angleFromReefCenter = angleFromReefCenter.plus(Rotation2d.k180deg).times(-1.0);
        Logger.recordOutput("Drive/GoalPoseAngle", angleFromReefCenter);
        return angleFromReefCenter;
    }

    /* Sets the goal using a command, meant to be used with buttonboard */
    public static Command setGoalCommand(Pose2d goalPose) {
        return Commands.runOnce(()-> customGoal = goalPose);
    }

    public static Command setSideCommand(SIDE reefSide) {
        return Commands.runOnce(() -> side = reefSide);
    } 

    private static boolean inBetween(double min, double max, double val) {
        return (val > min) && (val < max);
    }
}