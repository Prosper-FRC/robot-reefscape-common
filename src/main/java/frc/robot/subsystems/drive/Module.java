package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import static frc.robot.subsystems.drive.DriveConstants.kModuleControllerConfigs;

public class Module {
    public static final LoggedTunableNumber kDriveP = new LoggedTunableNumber(
        "Module/Drive/kP", kModuleControllerConfigs.driveController().getP());
    public static final LoggedTunableNumber kDriveD = new LoggedTunableNumber(
        "Module/Drive/kD", kModuleControllerConfigs.driveController().getD());
    public static final LoggedTunableNumber kDriveS = new LoggedTunableNumber(
        "Module/Drive/kS", kModuleControllerConfigs.driveFF().getKs());
    public static final LoggedTunableNumber kDriveV = new LoggedTunableNumber(
        "Module/Drive/kV", kModuleControllerConfigs.driveFF().getKv());
    public static final LoggedTunableNumber kDriveA = new LoggedTunableNumber(
        "Module/Drive/kA", kModuleControllerConfigs.driveFF().getKa());

    public static final LoggedTunableNumber kTurnP = new LoggedTunableNumber(
        "Module/AzimuthP", kModuleControllerConfigs.azimuthController().getP());
    public static final LoggedTunableNumber kTurnD = new LoggedTunableNumber(
        "Module/AzimuthD", kModuleControllerConfigs.azimuthController().getD());
    public static final LoggedTunableNumber kTurnS = new LoggedTunableNumber(
        "Module/AzimuthS", kModuleControllerConfigs.azimuthFF().getKs());

    private ModuleIO io;
    private ModuleInputsAutoLogged inputs = new ModuleInputsAutoLogged();

    private Double velocitySetpointMPS = null;
    private Double accelerationSetpointMPSS = null;
    // kDriveA ACTS AS A FUDGE FACTOR, NOT ACTUAL CONSTANT FOR AMPERAGE TUNING
    private Double amperageSetpoint = null;

    private Rotation2d azimuthSetpointAngle = null;

    private SwerveModuleState currentState = new SwerveModuleState();
    private SwerveModulePosition currentPosition = new SwerveModulePosition();

    private final String kLogKey;
    private SimpleMotorFeedforward driveFF = DriveConstants.kModuleControllerConfigs.driveFF();
    // Currently unused
    private SimpleMotorFeedforward azimuthFF = DriveConstants.kModuleControllerConfigs.azimuthFF();

    public Module(String key, ModuleIO io) {
        this.io = io;
        kLogKey = "Module/" + key;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/"+kLogKey, inputs);

        currentState = new SwerveModuleState(inputs.driveVelocityMPS, inputs.azimuthPosition);
        currentPosition = new SwerveModulePosition(inputs.drivePositionM, inputs.azimuthPosition);

        // Runs drive PID
        // Amperage and acceleration are never not null at the same time
        // Amperage is the FOC feedforward, and acceleration is the voltage feedforward
        if (velocitySetpointMPS != null) {
            Logger.recordOutput("Drive/"+kLogKey+"/velocitySepointMPS", velocitySetpointMPS);
            if(amperageSetpoint != null) {
                double ffOutput = driveFF.calculate(velocitySetpointMPS, amperageSetpoint);
                io.setDriveVelocity(velocitySetpointMPS, ffOutput);
                Logger.recordOutput("Drive/"+kLogKey+"/AmperageFeedforward", amperageSetpoint);
            } else if(accelerationSetpointMPSS != null) {
                double ffOutput = driveFF.calculate(velocitySetpointMPS, accelerationSetpointMPSS);
                Logger.recordOutput("Drive/"+kLogKey+"/SimpleFeedforward", ffOutput);
                io.setDriveVelocity(velocitySetpointMPS, ffOutput);
            } else {
                io.setDriveVelocity(velocitySetpointMPS, 0.0);
            }
        }

        // Runs azimuth PID
        if (azimuthSetpointAngle != null) {
            double ffOutput = azimuthFF.getKs() * Math.signum(inputs.azimuthVelocity.getDegrees());
            Logger.recordOutput("Drive/"+kLogKey+"/SimpleFeedforward", ffOutput);
            io.setAzimuthPosition(azimuthSetpointAngle, ffOutput);
        }

        // Updates PID
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                io.setDrivePID(kDriveP.get(), 0.0, kDriveD.get());
            }, kDriveP, kDriveD);

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                driveFF = new SimpleMotorFeedforward(kDriveS.get(), kDriveV.get(), kDriveA.get());
            }, kDriveS, kDriveV, kDriveA);

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                io.setAzimuthPID(kTurnP.get(), 0.0, kTurnD.get());
            }, kTurnP, kTurnD);

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                azimuthFF = new SimpleMotorFeedforward(kTurnS.get(), 0.0, 0.0);
            }, kTurnS);
    }

    public void runCharacterization(double inputVolts) {
        runCharacterization(inputVolts, new Rotation2d());
    }

    /* Runs characterization of by setting motor voltage and the rotation the module at a specific rotation */
    public void runCharacterization(double inputVolts, Rotation2d rot) {
        setDesiredRotation(rot);
        setDesiredVelocity(null);
        setDriveVoltage(inputVolts);
    }

    public SwerveModuleState setDesiredState(SwerveModuleState state) {
        setDesiredAcceleration(null);
        setDesiredAmperage(null);
        setDesiredVelocity(state.speedMetersPerSecond);
        setDesiredRotation(state.angle);
        return getDesiredState();
    }

    public SwerveModuleState setDesiredStateWithAmperage(SwerveModuleState state, Double desiredAmperage) {
        // kDriveA ACTS AS A FUDGE FACTOR, NOT ACTUAL CONSTANT FOR AMPERAGE TUNING
        setDesiredAmperage(desiredAmperage);
        setDesiredVelocity(state.speedMetersPerSecond);
        setDesiredRotation(state.angle);
        return getDesiredState();
    }

    public SwerveModuleState setDesiredStateWithAccel(SwerveModuleState state, Double accelMPSS) {
        setDesiredAcceleration(accelMPSS);
        setDesiredVelocity(state.speedMetersPerSecond);
        setDesiredRotation(state.angle);
        return getDesiredState();
    }

    public void setDesiredVelocity(Double velocitySetpoint) {
        velocitySetpointMPS = velocitySetpoint;
    }

    public void setDesiredAmperage(Double torqueNM) {
        this.accelerationSetpointMPSS = null;
        this.amperageSetpoint = torqueNM;
    }

    public void setDesiredAcceleration(Double accelerationMPSS) {
        this.amperageSetpoint = null;
        this.accelerationSetpointMPSS = accelerationMPSS;
    }

    public void setDesiredRotation(Rotation2d angleSetpoint) {
        azimuthSetpointAngle = angleSetpoint;
    }

    public void setDriveVoltage(double driveVolts) {
        io.setDriveVolts(driveVolts);
    }

    public void setDriveAmp(double amps) {
        io.setDriveAmperage(amps);
    }

    public void setAzimuthVoltage(double azimuthVolts) {
        io.setAzimuthVolts(azimuthVolts);
    }

    public SwerveModuleState getDesiredState() {
        return new SwerveModuleState(velocitySetpointMPS, azimuthSetpointAngle);
    }

    public SwerveModuleState getCurrentState() {
        return currentState;
    }

    public SwerveModulePosition getCurrentPosition() {
        return currentPosition;
    }

    public ModuleInputsAutoLogged getInputs() {
        return inputs;
    }

    public void stop() {
        io.setDriveVolts(0.0);
        io.setAzimuthVolts(0.0);
    }

    public void resetAzimuthEncoder() {
        io.resetAzimuthEncoder();
    }
}
