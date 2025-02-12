package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import static frc.robot.subsystems.drive.DriveConstants.kModuleControllerConfigs;

public class Module {
    public static final LoggedTunableNumber driveP = new LoggedTunableNumber(
        "Module/Drive/kP", kModuleControllerConfigs.driveController().getP());
    public static final LoggedTunableNumber driveD = new LoggedTunableNumber(
        "Module/Drive/kD", kModuleControllerConfigs.driveController().getD());
    public static final LoggedTunableNumber driveS = new LoggedTunableNumber(
        "Module/Drive/kS", kModuleControllerConfigs.driveFF().getKs());
    public static final LoggedTunableNumber driveV = new LoggedTunableNumber(
        "Module/Drive/kV", kModuleControllerConfigs.driveFF().getKv());
    public static final LoggedTunableNumber driveA = new LoggedTunableNumber(
        "Module/Drive/kA", kModuleControllerConfigs.driveFF().getKa());

    public static final LoggedTunableNumber turnP = new LoggedTunableNumber(
        "Module/AzimuthP", kModuleControllerConfigs.azimuthController().getP());
    public static final LoggedTunableNumber turnD = new LoggedTunableNumber(
        "Module/AzimuthD", kModuleControllerConfigs.azimuthController().getD());
    public static final LoggedTunableNumber turnS = new LoggedTunableNumber(
        "Module/AzimuthS", kModuleControllerConfigs.azimuthFF().getKs());

    private ModuleIO io;
    private ModuleInputsAutoLogged inputs = new ModuleInputsAutoLogged();

    private final String kLogKey;

    /* Drive Control */
    private Double velocitySetpointMPS = null;
    // kDriveA ACTS AS A FUDGE FACTOR, NOT ACTUAL CONSTANT FOR AMPERAGE TUNING
    private Double amperageSetpoint = null;
    private SimpleMotorFeedforward driveFF = DriveConstants.kModuleControllerConfigs.driveFF();

    /* Azimuth Control */
    private Rotation2d azimuthSetpointAngle = null;
    private SimpleMotorFeedforward azimuthFF = DriveConstants.kModuleControllerConfigs.azimuthFF();

    private SwerveModuleState currentState = new SwerveModuleState();
    private SwerveModulePosition currentPosition = new SwerveModulePosition();

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
        // if Amperage is null no feedforward is used
        // Amperage is the FOC feedforward
        if (velocitySetpointMPS != null) {
            Logger.recordOutput("Drive/"+kLogKey+"/velocitySepointMPS", velocitySetpointMPS);
            if(amperageSetpoint != null) {
                double ffOutput = driveFF.calculate(velocitySetpointMPS, amperageSetpoint);

                Logger.recordOutput("Drive/"+kLogKey+"/AmperageFeedforward", amperageSetpoint);
                Logger.recordOutput("Drive/"+kLogKey+"/ffOutput", ffOutput);

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

        // Updates PID values for drive and azimuth
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                io.setDrivePID(driveP.get(), 0.0, driveD.get());
            }, driveP, driveD);

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                driveFF = new SimpleMotorFeedforward(driveS.get(), driveV.get(), driveA.get());
            }, driveS, driveV, driveA);

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                io.setAzimuthPID(turnP.get(), 0.0, turnD.get());
            }, turnP, turnD);

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                azimuthFF = new SimpleMotorFeedforward(turnS.get(), 0.0, 0.0);
            }, turnS);
    }

    /* Sets the desired setpoint of the module without FF */
    public SwerveModuleState setDesiredState(SwerveModuleState state) {
        setDesiredAmperage(null);
        setDesiredVelocity(state.speedMetersPerSecond);
        setDesiredRotation(state.angle);
        return getDesiredState();
    }

    /* Sets the desired setpoint of the module with FF */
    public SwerveModuleState setDesiredStateWithAmperage(SwerveModuleState state, Double desiredAmperage) {
        // kDriveA ACTS AS A FUDGE FACTOR, NOT ACTUAL CONSTANT FOR AMPERAGE TUNING
        setDesiredAmperage(desiredAmperage);
        setDesiredVelocity(state.speedMetersPerSecond);
        setDesiredRotation(state.angle);
        return getDesiredState();
    }

    /* Runs characterization of by setting motor drive voltage and rotates the module forward */
    public void runCharacterization(double inputVolts) {
        runCharacterization(inputVolts, new Rotation2d());
    }

    /* Runs characterization of by setting motor drive voltage and the rotation the module at a specific rotation */
    public void runCharacterization(double inputVolts, Rotation2d rot) {
        setDesiredRotation(rot);
        setDesiredVelocity(null);
        setDriveVoltage(inputVolts);
    }

    /* Sets the velocity of the module */
    public void setDesiredVelocity(Double velocitySetpoint) {
        velocitySetpointMPS = velocitySetpoint;
    }

    /* Sets the amperage(FOC) feedforward */
    public void setDesiredAmperage(Double amperage) {
        this.amperageSetpoint = amperage;
    }

    /* Sets azimuth rotation goal */
    public void setDesiredRotation(Rotation2d angleSetpoint) {
        azimuthSetpointAngle = angleSetpoint;
    }

    /* Sets drive voltage */
    public void setDriveVoltage(double driveVolts) {
        io.setDriveVolts(driveVolts);
    }

    /* Sets drive amperage using FOC */
    public void setDriveAmp(double amps) {
        io.setDriveAmperage(amps);
    }

    /* Set azimuth voltage */
    public void setAzimuthVoltage(double azimuthVolts) {
        io.setAzimuthVolts(azimuthVolts);
    }

    /* Stops modules by setting voltage to zero */
    public void stop() {
        setDriveVoltage(0.0);
        setAzimuthVoltage(0.0);
    }

    /* Gets the setpoint state of the module(speed and rotation) */
    public SwerveModuleState getDesiredState() {
        return new SwerveModuleState(velocitySetpointMPS, azimuthSetpointAngle);
    }

    /* Gets the physical state of the module(speed and rotation) */
    public SwerveModuleState getCurrentState() {
        return currentState;
    }

    /* Gets the physical position of the module(position and rotation) */
    public SwerveModulePosition getCurrentPosition() {
        return currentPosition;
    }

    /* All logged hardware data in the module */
    public ModuleInputsAutoLogged getInputs() {
        return inputs;
    }

    /* Resets azimuth encoder from CANCoder */
    public void resetAzimuthEncoder() {
        io.resetAzimuthEncoder();
    }
}
