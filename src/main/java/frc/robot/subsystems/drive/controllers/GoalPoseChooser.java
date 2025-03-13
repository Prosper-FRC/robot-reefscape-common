package frc.robot.subsystems.drive.controllers;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
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
    // @AutoLogOutput(key="GoalPoseChooser/Side")
    private static SIDE side = SIDE.RIGHT;

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
        Rotation2d angleFromReefCenter = turnFromReefOriginForHexagon(robotPose);
        Pose2d goal;
        if(inBetween(-30.0, 30.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "D");
            if(side.equals(SIDE.LEFT)) {
                goal = FieldConstants.DL;
            } else goal = FieldConstants.DR;
        } else if(inBetween(30.0, 90.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "E");
            if(side.equals(SIDE.LEFT)) {
                goal = FieldConstants.EL;
            } else goal = FieldConstants.ER;
        } else if(inBetween(90.0, 150.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "F");
            if(side.equals(SIDE.LEFT)) {
                goal = FieldConstants.FL;
            } else goal = FieldConstants.FR;
            // Skipped -150 to 150 because the inBetween function miscopes
            // Putting it in else covers the remainder of the hexagon scope
        } else if(inBetween(-150.0, -90.0, angleFromReefCenter.getDegrees())) {
            Logger.recordOutput("Drive/ReefSide", "B");
            if(side.equals(SIDE.LEFT)) {
                goal = FieldConstants.BL;
            } else goal = FieldConstants.BR;
        } else if(inBetween(-90.0, -30.0, angleFromReefCenter.getDegrees())){
            Logger.recordOutput("Drive/ReefSide", "C");
            if(side.equals(SIDE.LEFT)) {
                goal = FieldConstants.CL;
            } else goal = FieldConstants.CR;
        } else {
            Logger.recordOutput("Drive/ReefSide", "A");
            if(side.equals(SIDE.LEFT)) {
                goal = FieldConstants.AL;
            } else goal = FieldConstants.AR;
        }
        // Logger.recordOutput("Drive/SelectedSide", side);

        return AllianceFlipUtil.apply(goal);
    }

    public static void updateSideStuff() {
        Logger.recordOutput("Drive/SelectedSide", side);
    }

    public static Pose2d getIntakePose(Pose2d robotPose) {
        if(DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            return AllianceFlipUtil.apply((robotPose.getY() < Constants.kFieldWidthMeters / 2.0) ? FieldConstants.blueIR : FieldConstants.blueIL);
        } else return (robotPose.getY() < Constants.kFieldWidthMeters / 2.0) ? FieldConstants.redIL : FieldConstants.redIR;
    }

    /* DO NOT USE X COORDINATE, REPLACE y holonomic speeds with driver controller when using this! */
    public static Pose2d getNetPose(Pose2d robotPose) {
        return AllianceFlipUtil.apply(new Pose2d(FieldConstants.kXNetLineMeters, 0.0, Rotation2d.kZero));
    }

    /* Accoumts for rotation from reef, and offsets for red-side logic */
    public static Rotation2d turnFromReefOriginForHexagon(Pose2d robotPose) {
        Pose2d reefCenter = AllianceFlipUtil.apply(FieldConstants.kReefCenter);
        Rotation2d angleFromReefCenter = Rotation2d.fromRadians(
            Math.atan2(
                robotPose.getY() - reefCenter.getY(), 
                robotPose.getX() - reefCenter.getX()));

        Rotation2d finalAngle = angleFromReefCenter.times(-1.0);
        if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) finalAngle = angleFromReefCenter.plus(Rotation2d.k180deg).times(-1.0);
        Logger.recordOutput("Drive/GoalPoseAngle", finalAngle);
        return finalAngle;
    }

    /* Accoumts for rotation from reef, and offsets for red-side logic */
    public static Rotation2d turnFromReefOrigin(Pose2d robotPose) {
        Pose2d reefCenter = AllianceFlipUtil.apply(FieldConstants.kReefCenter);
        Rotation2d angleFromReefCenter = Rotation2d.fromRadians(
            Math.atan2(
                robotPose.getY() - reefCenter.getY(), 
                robotPose.getX() - reefCenter.getX()));
        if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) angleFromReefCenter = angleFromReefCenter.plus(Rotation2d.k180deg);
        Logger.recordOutput("Drive/GoalPoseAngle", angleFromReefCenter);
        return angleFromReefCenter;
    }

    /* Algae align math */
    public static double findDistanceFromIntersection(int reefAngle, Pose2d robotPose) {
        // To find slope of the line
        double m = Math.tan(reefAngle);

        // Poses of center of reef 
        // TODO: Update with values from Choreo later
        double xVal = 0.0;
        double yVal = 0.0;

        // Robot poses
        double xPose = robotPose.getX();
        double yPose = robotPose.getY();


        // Point slope form: m (x - xVal) - (y - yVal)
        double equationOfReefLine = (-m * xPose) + (yPose) + (m * (xVal - yVal));
        double A = - m;
        double C = m * (xVal - yVal);

        // Plugging in constants A, B (which will always be 1) and C from standard form into equation
        // for calculating distance from a point to line
        double distanceOfP =
        (Math.abs((A * xPose) + yPose + C)) /
        Math.sqrt(Math.pow(A, 2) + 1);

        double perpBisector = // do i just flip the x and y of the equationOfReefLine? cus that would yield the line of the perp bisector

        // PIDController pid = new PIDController(A, C, distanceOfP);
        // pid.calculate(robotPose.getX(), distanceOfP);

        return distanceOfP;
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