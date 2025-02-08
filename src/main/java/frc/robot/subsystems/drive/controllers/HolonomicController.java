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
    public static final LoggedTunableNumber kXP = new LoggedTunableNumber(
        "AutoAlign/X/kP", 1.0);
    public static final LoggedTunableNumber kXD = new LoggedTunableNumber(
        "AutoAlign/X/kD", 0.0);
    public static final LoggedTunableNumber kXI = new LoggedTunableNumber(
        "AutoAlign/X/kI", 0.0);
    public static final LoggedTunableNumber kXIZone = new LoggedTunableNumber(
        "AutoAlign/X/kIZone", 0.0);
    public static final LoggedTunableNumber kXIRange = new LoggedTunableNumber(
        "AutoAlign/X/kIRange", 0.0);
    public static final LoggedTunableNumber kXMaxVMPS = new LoggedTunableNumber(
        "AutoAlign/X/kMaxVMPS", 3.0);
    public static final LoggedTunableNumber kXMaxAMPSS = new LoggedTunableNumber(
        "AutoAlign/X/kMaxVMPSS", 9.0);

    public static final LoggedTunableNumber kXS = new LoggedTunableNumber(
        "AutoAlign/X/kS", 0.0);
    public static final LoggedTunableNumber kXV = new LoggedTunableNumber(
        "AutoAlign/X/kV", 1.0);

    public static final LoggedTunableNumber kXToleranceMeters = new LoggedTunableNumber(
        "AutoAlign/X/ToleranceMeters", 0.01);

    public static final LoggedTunableNumber kYP = new LoggedTunableNumber(
        "AutoAlign/Y/kP", 1.0);
    public static final LoggedTunableNumber kYD = new LoggedTunableNumber(
        "AutoAlign/Y/kD", 0.0);
    public static final LoggedTunableNumber kYI = new LoggedTunableNumber(
        "AutoAlign/Y/kI", 0.0);  
    public static final LoggedTunableNumber kYIZone = new LoggedTunableNumber(
        "AutoAlign/Y/kIZone", 0.1);
    public static final LoggedTunableNumber kYIRange = new LoggedTunableNumber(
        "AutoAlign/Y/kIRange", 0.0);
    public static final LoggedTunableNumber kYMaxVMPS = new LoggedTunableNumber(
        "AutoAlign/Y/kMaxVMPS", 3.0);
    public static final LoggedTunableNumber kYMaxAMPSS = new LoggedTunableNumber(
        "AutoAlign/Y/kMaxVMPSS", 9.0);

    public static final LoggedTunableNumber kYS = new LoggedTunableNumber(
        "AutoAlign/Y/kS", 0.0);
    public static final LoggedTunableNumber kYV = new LoggedTunableNumber(
        "AutoAlign/Y/kV", 1.0);

    public static final LoggedTunableNumber kYToleranceMeters = new LoggedTunableNumber(
        "AutoAlign/Y/ToleranceMeters", 0.01);

    public static final LoggedTunableNumber kOmegaP = new LoggedTunableNumber(
        "AutoAlign/Omega/kP", 1.0);
    public static final LoggedTunableNumber kOmegaD = new LoggedTunableNumber(
        "AutoAlign/Omega/kD", 0.0);

    public static final LoggedTunableNumber kOmegaI = new LoggedTunableNumber(
        "AutoAlign/Omega/kI", 0.0);  
    public static final LoggedTunableNumber kOmegaIZone = new LoggedTunableNumber(
        "AutoAlign/Omega/kIZone", 0.0);
    public static final LoggedTunableNumber kOmegaIRange = new LoggedTunableNumber(
        "AutoAlign/Omega/kIRange", 0.0);

    public static final LoggedTunableNumber kOmegaMaxVDPS = new LoggedTunableNumber(
        "AutoAlign/Omega/kMaxVDPS", 180);
    public static final LoggedTunableNumber kOmegaMaxADPSS = new LoggedTunableNumber(
        "AutoAlign/Omega/kMaxVDPSS", 1500);

    public static final LoggedTunableNumber kOmegaS = new LoggedTunableNumber(
        "AutoAlign/Omega/kS", 0.0);
    public static final LoggedTunableNumber kOmegaV = new LoggedTunableNumber(
        "AutoAlign/Omega/kV", 1.0);

    public static final LoggedTunableNumber kOmegaToleranceDegrees = new LoggedTunableNumber(
        "AutoAlign/Omega/ToleranceDegrees", 1.0);

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController omegaController;

    private SimpleMotorFeedforward xFeedforward;
    private SimpleMotorFeedforward yFeedforward;
    private SimpleMotorFeedforward omegaFeedforward;

    public HolonomicController() {
        this.xController = new ProfiledPIDController(kXP.get(), kXI.get(), kXD.get(), new Constraints(kXMaxVMPS.get(), kXMaxAMPSS.get()));
        xController.setIntegratorRange(-kXIRange.get(), kXIRange.get());
        xController.setIZone(kXIZone.get());
        xController.setConstraints(new Constraints(kXMaxVMPS.get(), kXMaxAMPSS.get()));
        xController.setTolerance(kXToleranceMeters.get());
        this.xFeedforward = new SimpleMotorFeedforward(kXS.get(), kXV.get());

        this.yController = new ProfiledPIDController(kYP.get(), kYI.get(), kYD.get(), new Constraints(kYMaxVMPS.get(), kYMaxAMPSS.get()));
        yController.setIntegratorRange(-kYIRange.get(), kYIRange.get());
        yController.setIZone(kYIZone.get());
        yController.setConstraints(new Constraints(kYMaxVMPS.get(), kYMaxAMPSS.get()));
        yController.setTolerance(kYToleranceMeters.get());
        this.yFeedforward = new SimpleMotorFeedforward(kYS.get(), kYV.get());

        this.omegaController = new ProfiledPIDController(kOmegaP.get(), kOmegaI.get(), kOmegaD.get(), new Constraints(kOmegaMaxVDPS.get(), kOmegaMaxADPSS.get()));
        omegaController.enableContinuousInput( -180.0, 180.0 );
        omegaController.setIntegratorRange(-kOmegaIRange.get(), kOmegaIRange.get());
        omegaController.setIZone(kOmegaIZone.get());
        omegaController.setConstraints(new Constraints(kOmegaMaxVDPS.get(), kOmegaMaxADPSS.get()));
        omegaController.setTolerance(kOmegaToleranceDegrees.get());
        this.omegaFeedforward = new SimpleMotorFeedforward(kOmegaS.get(), kOmegaV.get());
    }

    public void reset(Pose2d startPose) {
        reset( startPose, new ChassisSpeeds() );
    }

    /* Resets the robot based on the position and the speed of the robot 
     * Resetting with the speed allows the robot to stay controlled
     * while the driver is moving
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
            xController.calculate( 
                currentPose.getX(), 
                new TrapezoidProfile.State(
                    goalPose.getX(),
                    goalSpeed.vxMetersPerSecond) )
            + xFeedforward.calculate(xController.getSetpoint().velocity),

            yController.calculate( 
                currentPose.getY(), 
                new TrapezoidProfile.State(
                    goalPose.getY(),
                    goalSpeed.vyMetersPerSecond) )
            + yFeedforward.calculate(yController.getSetpoint().velocity),

            Math.toRadians (omegaController.calculate( 
                currentPose.getRotation().getDegrees(), 
                new TrapezoidProfile.State(
                    goalPose.getRotation().getDegrees(),
                    Math.toDegrees(goalSpeed.omegaRadiansPerSecond) ) )
             + omegaFeedforward.calculate(omegaController.getSetpoint().velocity)),
             
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
            xController.setPID(kXP.get(), kXI.get(), kXD.get());
            xController.setIntegratorRange(-kXIRange.get(), kXIRange.get());
            xController.setIZone(kXIZone.get());
            xController.setConstraints(new Constraints(kXMaxVMPS.get(), kXMaxAMPSS.get()));
        }, kXP, kXI, kXD, kXIRange, kXIZone, kXMaxVMPS, kXMaxAMPSS);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            xFeedforward = new SimpleMotorFeedforward(kXS.get(), kXV.get());
        }, kXS, kXV);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            yController.setPID(kYP.get(), kYI.get(), kYD.get());
            yController.setIntegratorRange(-kYIRange.get(), kYIRange.get());
            yController.setIZone(kYIZone.get());
            xController.setConstraints(new Constraints(kYMaxVMPS.get(), kYMaxAMPSS.get()));
        }, kYP, kYI, kYD, kYIRange, kYIZone, kYMaxVMPS, kYMaxAMPSS);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            yFeedforward = new SimpleMotorFeedforward(kYS.get(), kYV.get());
        }, kYS, kYV);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            omegaController.setPID(kOmegaP.get(), kOmegaI.get(), kOmegaD.get());
            omegaController.setIntegratorRange(-kOmegaIRange.get(), kOmegaIRange.get());
            omegaController.setIZone(kOmegaIZone.get());
            omegaController.setConstraints(new Constraints(kOmegaMaxVDPS.get(), kOmegaMaxADPSS.get()));
        }, kOmegaP, kOmegaI, kOmegaD, kOmegaIRange, kOmegaIZone, kOmegaMaxVDPS, kOmegaMaxADPSS);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            omegaFeedforward = new SimpleMotorFeedforward(kOmegaS.get(), kOmegaV.get());
        }, kOmegaS, kOmegaV);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            xController.setTolerance( kXToleranceMeters.get() );
            yController.setTolerance( kYToleranceMeters.get() );
            omegaController.setTolerance( kOmegaToleranceDegrees.get() );
        }, kXToleranceMeters, kYToleranceMeters, kOmegaToleranceDegrees);
    }
}