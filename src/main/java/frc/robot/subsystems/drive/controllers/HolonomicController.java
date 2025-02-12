package frc.robot.subsystems.drive.controllers;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.utils.debugging.LoggedTunableNumber;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class HolonomicController {
    public static final LoggedTunableNumber xP = new LoggedTunableNumber(
        "AutoAlign/X/kP", 2.0);
    public static final LoggedTunableNumber xD = new LoggedTunableNumber(
        "AutoAlign/X/kD", 0.0);
    public static final LoggedTunableNumber xI = new LoggedTunableNumber(
        "AutoAlign/X/kI", 0.0);
    public static final LoggedTunableNumber xIZone = new LoggedTunableNumber(
        "AutoAlign/X/kIZone", 0.0);
    public static final LoggedTunableNumber xIRange = new LoggedTunableNumber(
        "AutoAlign/X/kIRange", 0.0);
    public static final LoggedTunableNumber xMaxVMPS = new LoggedTunableNumber(
        "AutoAlign/X/kMaxVMPS", 3.0);
    public static final LoggedTunableNumber xMaxAMPSS = new LoggedTunableNumber(
        "AutoAlign/X/kMaxVMPSS", 3.0);

    public static final LoggedTunableNumber xS = new LoggedTunableNumber(
        "AutoAlign/X/kS", 0.0);
    public static final LoggedTunableNumber xV = new LoggedTunableNumber(
        "AutoAlign/X/kV", 0.5);

    public static final LoggedTunableNumber xToleranceMeters = new LoggedTunableNumber(
        "AutoAlign/X/ToleranceMeters", 0.01);

    public static final LoggedTunableNumber yP = new LoggedTunableNumber(
        "AutoAlign/Y/kP", 1.0);
    public static final LoggedTunableNumber yD = new LoggedTunableNumber(
        "AutoAlign/Y/kD", 0.0);
    public static final LoggedTunableNumber yI = new LoggedTunableNumber(
        "AutoAlign/Y/kI", 0.0);  
    public static final LoggedTunableNumber yIZone = new LoggedTunableNumber(
        "AutoAlign/Y/kIZone", 0.0);
    public static final LoggedTunableNumber yIRange = new LoggedTunableNumber(
        "AutoAlign/Y/kIRange", 0.0);
    public static final LoggedTunableNumber yMaxVMPS = new LoggedTunableNumber(
        "AutoAlign/Y/kMaxVMPS", 3.0);
    public static final LoggedTunableNumber yMaxAMPSS = new LoggedTunableNumber(
        "AutoAlign/Y/kMaxVMPSS", 3.0);

    public static final LoggedTunableNumber yS = new LoggedTunableNumber(
        "AutoAlign/Y/kS", 0.0);
    public static final LoggedTunableNumber yV = new LoggedTunableNumber(
        "AutoAlign/Y/kV", 0.5);

    public static final LoggedTunableNumber yToleranceMeters = new LoggedTunableNumber(
        "AutoAlign/Y/ToleranceMeters", 0.01);

    public static final LoggedTunableNumber omegaP = new LoggedTunableNumber(
        "AutoAlign/Omega/kP", 1.0);
    public static final LoggedTunableNumber omegaD = new LoggedTunableNumber(
        "AutoAlign/Omega/kD", 0.0);

    public static final LoggedTunableNumber omegaI = new LoggedTunableNumber(
        "AutoAlign/Omega/kI", 0.0);  
    public static final LoggedTunableNumber omegaIZone = new LoggedTunableNumber(
        "AutoAlign/Omega/kIZone", 0.0);
    public static final LoggedTunableNumber omegaIRange = new LoggedTunableNumber(
        "AutoAlign/Omega/kIRange", 0.0);

    public static final LoggedTunableNumber omegaMaxVDPS = new LoggedTunableNumber(
        "AutoAlign/Omega/kMaxVDPS", 180);
    public static final LoggedTunableNumber omegaMaxADPSS = new LoggedTunableNumber(
        "AutoAlign/Omega/kMaxVDPSS", 180);

    public static final LoggedTunableNumber omegaS = new LoggedTunableNumber(
        "AutoAlign/Omega/kS", 0.0);
    public static final LoggedTunableNumber omegaV = new LoggedTunableNumber(
        "AutoAlign/Omega/kV", 0.5);

    public static final LoggedTunableNumber omegaToleranceDegrees = new LoggedTunableNumber(
        "AutoAlign/Omega/ToleranceDegrees", 1.0);

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController omegaController;

    private SimpleMotorFeedforward xFeedforward;
    private SimpleMotorFeedforward yFeedforward;
    private SimpleMotorFeedforward omegaFeedforward;

    public HolonomicController() {
        this.xController = new ProfiledPIDController(xP.get(), xI.get(), xD.get(), new Constraints(xMaxVMPS.get(), xMaxAMPSS.get()));
        xController.setIntegratorRange(-xIRange.get(), xIRange.get());
        xController.setIZone(xIZone.get());
        xController.setConstraints(new Constraints(xMaxVMPS.get(), xMaxAMPSS.get()));
        xController.setTolerance(xToleranceMeters.get());
        this.xFeedforward = new SimpleMotorFeedforward(xS.get(), xV.get());

        this.yController = new ProfiledPIDController(yP.get(), yI.get(), yD.get(), new Constraints(yMaxVMPS.get(), yMaxAMPSS.get()));
        yController.setIntegratorRange(-yIRange.get(), yIRange.get());
        yController.setIZone(yIZone.get());
        yController.setConstraints(new Constraints(yMaxVMPS.get(), yMaxAMPSS.get()));
        yController.setTolerance(yToleranceMeters.get());
        this.yFeedforward = new SimpleMotorFeedforward(yS.get(), yV.get());

        this.omegaController = new ProfiledPIDController(omegaP.get(), omegaI.get(), omegaD.get(), new Constraints(omegaMaxVDPS.get(), omegaMaxADPSS.get()));
        omegaController.enableContinuousInput( -180.0, 180.0 );
        omegaController.setIntegratorRange(-omegaIRange.get(), omegaIRange.get());
        omegaController.setIZone(omegaIZone.get());
        omegaController.setConstraints(new Constraints(omegaMaxVDPS.get(), omegaMaxADPSS.get()));
        omegaController.setTolerance(omegaToleranceDegrees.get());
        this.omegaFeedforward = new SimpleMotorFeedforward(omegaS.get(), omegaV.get());
    }

    public void reset(Pose2d startPose) {
        reset( startPose, new ChassisSpeeds() );
    }

    /* 
     * Resets the robot based on the position and the speed of the robot 
     * Resetting with the speed allows the robot to stay controlled
     * while the driver is moving before drive to pose is activated
     */
    public void reset(Pose2d robotPose, ChassisSpeeds robotChassisSpeeds) {
        xController.reset( 
            new State(
                robotPose.getX(),
                robotChassisSpeeds.vxMetersPerSecond ) );

        yController.reset( 
            new State(
                robotPose.getY(),
                robotChassisSpeeds.vyMetersPerSecond ) );

        omegaController.reset(
            new State(
                robotPose.getRotation().getDegrees(),
                Math.toDegrees(robotChassisSpeeds.omegaRadiansPerSecond) ) );
    }

    /* Sets the goals of the controllers. Is not needed for current code */
    public void setGoal(Pose2d goalPose) {
        setGoal(goalPose, new ChassisSpeeds());
    }

    public void setGoal(Pose2d goalPose, ChassisSpeeds goalSpeed) {
        xController.setGoal( 
            new TrapezoidProfile.State(
                goalPose.getX(), 
                goalSpeed.vxMetersPerSecond) );
        yController.setGoal( 
            new TrapezoidProfile.State(
                goalPose.getY(),
                goalSpeed.vyMetersPerSecond) );
        omegaController.setGoal( 
            new TrapezoidProfile.State(
                goalPose.getRotation().getRadians(),
                goalSpeed.omegaRadiansPerSecond) );
    }

    public ChassisSpeeds calculate(Pose2d goalPose, Pose2d currentPose) {
        return calculate(goalPose, new ChassisSpeeds(), currentPose);
    }

    /* Uses 3 PID controllers to set the chassis speeds */
    public ChassisSpeeds calculate(Pose2d goalPose, ChassisSpeeds goalSpeed, Pose2d currentPose) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            (xController.calculate( 
                currentPose.getX(), 
                new TrapezoidProfile.State(
                    goalPose.getX(),
                    goalSpeed.vxMetersPerSecond) )
            + xFeedforward.calculate(xController.getSetpoint().velocity)),

            (yController.calculate( 
                currentPose.getY(), 
                new TrapezoidProfile.State(
                    goalPose.getY(),
                    goalSpeed.vyMetersPerSecond) )
            + yFeedforward.calculate(yController.getSetpoint().velocity)),

            (Math.toRadians (omegaController.calculate( 
                currentPose.getRotation().getDegrees(), 
                new TrapezoidProfile.State(
                    goalPose.getRotation().getDegrees(),
                    Math.toDegrees(goalSpeed.omegaRadiansPerSecond) ) )
             + omegaFeedforward.calculate(omegaController.getSetpoint().velocity))),
             
            currentPose.getRotation()
        );
    }

    ////////////////////////// GETTERS \\\\\\\\\\\\\\\\\\\\\\\\\\\\
    @AutoLogOutput(key = "Drive/HolonomicController/AtGoal")    
    public boolean atGoal() {
        return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
    }

    @AutoLogOutput(key = "Drive/HolonomicController/PositionGoal")
    public Pose2d getPositionGoal() {
        return new Pose2d(
            new Translation2d(
                xController.getGoal().position, 
                yController.getGoal().position), 
            Rotation2d.fromDegrees(
                omegaController.getGoal().position
            ));
    }

    @AutoLogOutput(key = "Drive/HolonomicController/PositionSetpoint")
    public Pose2d getPositionSetpoint() {
        return new Pose2d(
            new Translation2d(
                xController.getSetpoint().position, 
                yController.getSetpoint().position ), 
            Rotation2d.fromDegrees(
                omegaController.getSetpoint().position ) );
    }

    @AutoLogOutput(key = "Drive/HolonomicController/VelocityGoal")
    public ChassisSpeeds getVelocityGoal() {
        return new ChassisSpeeds(
            xController.getGoal().velocity,
            yController.getGoal().velocity,
            Math.toRadians( omegaController.getGoal().velocity ) );
    }

    @AutoLogOutput(key = "Drive/HolonomicController/VelocitySetpoint")
    public ChassisSpeeds getVelocitySetpoint() {
        return new ChassisSpeeds(
            xController.getSetpoint().velocity,
            yController.getSetpoint().velocity,
            Math.toRadians( omegaController.getSetpoint().velocity ) );
    }

    @AutoLogOutput(key = "Drive/HolonomicController/PoseError")
    public Pose2d getPoseError() {
        return new Pose2d(
            xController.getPositionError(),
            yController.getPositionError(),
            new Rotation2d(omegaController.getPositionError())
        );
    }

    ////////////////////////// SETTERS \\\\\\\\\\\\\\\\\\\\\\\\\\\\
    public void updateAlignmentControllers() {
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            xController.setPID(xP.get(), xI.get(), xD.get());
            xController.setIntegratorRange(-xIRange.get(), xIRange.get());
            xController.setIZone(xIZone.get());
            xController.setConstraints(new Constraints(xMaxVMPS.get(), xMaxAMPSS.get()));
        }, xP, xI, xD, xIRange, xIZone, xMaxVMPS, xMaxAMPSS);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            xFeedforward = new SimpleMotorFeedforward(xS.get(), xV.get());
        }, xS, xV);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            yController.setPID(yP.get(), yI.get(), yD.get());
            yController.setIntegratorRange(-yIRange.get(), yIRange.get());
            yController.setIZone(yIZone.get());
            xController.setConstraints(new Constraints(yMaxVMPS.get(), yMaxAMPSS.get()));
        }, yP, yI, yD, yIRange, yIZone, yMaxVMPS, yMaxAMPSS);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            yFeedforward = new SimpleMotorFeedforward(yS.get(), yV.get());
        }, yS, yV);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            omegaController.setPID(omegaP.get(), omegaI.get(), omegaD.get());
            omegaController.setIntegratorRange(-omegaIRange.get(), omegaIRange.get());
            omegaController.setIZone(omegaIZone.get());
            omegaController.setConstraints(new Constraints(omegaMaxVDPS.get(), omegaMaxADPSS.get()));
        }, omegaP, omegaI, omegaD, omegaIRange, omegaIZone, omegaMaxVDPS, omegaMaxADPSS);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            omegaFeedforward = new SimpleMotorFeedforward(omegaS.get(), omegaV.get());
        }, omegaS, omegaV);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            xController.setTolerance( xToleranceMeters.get() );
            yController.setTolerance( yToleranceMeters.get() );
            omegaController.setTolerance( omegaToleranceDegrees.get() );
        }, xToleranceMeters, yToleranceMeters, omegaToleranceDegrees);
    }
}