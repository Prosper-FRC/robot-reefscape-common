package frc.robot.subsystems.drive.controllers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/* Chooses pose based of strategy and psoe */ 
public class GoalPoseChooser {
    public static enum CHOOSER_STRATEGY {
        TEST,
        HEXAGONAL,
        BUTTON_BOARD
    }

    public static enum SIDE {
        LEFT, RIGHT
    }

    public static final Pose2d kALeft = new Pose2d(3.171, 4.163, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kARight = new Pose2d(3.171, 3.896, Rotation2d.fromDegrees(180.0));

    public static final Pose2d kBLeft = new Pose2d(3.696, 2.951, Rotation2d.fromDegrees(60.0));
    public static final Pose2d kBRight = new Pose2d(3.952, 2.815, Rotation2d.fromDegrees(60.0));

    public static final Pose2d kCLeft = new Pose2d(5.048, 2.815, Rotation2d.fromDegrees(120.0));
    public static final Pose2d kCRight = new Pose2d(5.304, 2.951, Rotation2d.fromDegrees(120.0));

    public static final Pose2d kDLeft = new Pose2d(5.815, 3.896, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kDRight = new Pose2d(5.815, 4.163, Rotation2d.fromDegrees(0.0));

    public static final Pose2d kELeft = new Pose2d(5.304, 5.049, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d kERight = new Pose2d(5.048, 5.185, Rotation2d.fromDegrees(-120.0));

    public static final Pose2d kFLeft = new Pose2d(3.696, 5.185, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d kFRight = new Pose2d(3.952, 5.086, Rotation2d.fromDegrees(-60.0));

    public static final Pose2d kReefCenter = new Pose2d(4.5, 4, Rotation2d.fromDegrees(0.0));

    private static Pose2d buttonBoardGoal = kALeft;
    private static SIDE side = SIDE.LEFT;

    public static Pose2d getGoalPose(CHOOSER_STRATEGY strategy, Pose2d pose) {
        switch(strategy) {
            case TEST:
                return new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(60));
            case HEXAGONAL:
                return getHexagonalPose(pose);
            case BUTTON_BOARD:
                return buttonBoardGoal;
        }
        return new Pose2d();
    }

    public static Pose2d getHexagonalPose(Pose2d robotPose) {
        Rotation2d angleFromReefCenter = Rotation2d.fromRadians(
            Math.atan2(
                robotPose.getY() - kReefCenter.getY(), 
                robotPose.getX() - kReefCenter.getX()));
        Logger.recordOutput("Drive/GoalPoseAngle", angleFromReefCenter);
        if(inBetween(-30.0, 30.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "A");
            if(side.equals(SIDE.LEFT)) {
                return kDLeft;
            } else return kDRight;
        } else if(inBetween(30.0, 90.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "B");
            if(side.equals(SIDE.LEFT)) {
                return kELeft;
            } else return kERight;
        } else if(inBetween(90.0, 150.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "C");
            if(side.equals(SIDE.LEFT)) {
                return kFLeft;
            } else return kFRight;
            // Skipped -150 to 150 because the inBetween function miscopes
            // Putting it in else covers the remainder of the hexagon scope
        } else if(inBetween(-150.0, -90.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "E");
            if(side.equals(SIDE.LEFT)) {
                return kBLeft;
            } else return kBRight;
            // 
        } else if(inBetween(-90.0, -30.0, angleFromReefCenter.getDegrees())){
            Logger.recordOutput("Drive/ReefSide", "F");
            if(side.equals(SIDE.LEFT)) {
                return kCLeft;
            } else return kCRight;
        } else {
            Logger.recordOutput("Drive/ReefSide", "D");
            if(side.equals(SIDE.LEFT)) {
                return kALeft;
            } else return kARight;
        }
    }

    public static Command setGoalCommand(Pose2d goalPose) {
        return Commands.runOnce(()-> buttonBoardGoal = goalPose);
    }

    private static boolean inBetween(double min, double max, double val) {
        return (val > min) && (val < max);
    }
}
