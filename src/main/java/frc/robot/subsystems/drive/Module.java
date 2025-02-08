package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.debugging.LoggedTunableNumber;

public class Module {

    public static final LoggedTunableNumber driveP = new LoggedTunableNumber("Module/Drive/kP", DriveConstants.kModuleControllerConfigs.driveController().getP());
    public static final LoggedTunableNumber driveD = new LoggedTunableNumber("Module/Drive/kD", DriveConstants.kModuleControllerConfigs.driveController().getD());
    public static final LoggedTunableNumber driveS = new LoggedTunableNumber("Module/Drive/kS", DriveConstants.kModuleControllerConfigs.driveFF().getKs());
    public static final LoggedTunableNumber driveV = new LoggedTunableNumber("Module/Drive/kV", DriveConstants.kModuleControllerConfigs.driveFF().getKv());
    public static final LoggedTunableNumber driveA = new LoggedTunableNumber("Modulkeye/Drive/kA", DriveConstants.kModuleControllerConfigs.driveFF().getKa());

    public static final LoggedTunableNumber azimuthP = new LoggedTunableNumber("Module/Azimuth/kP", DriveConstants.kModuleControllerConfigs.azimuthController().getP());
    public static final LoggedTunableNumber azimuthD = new LoggedTunableNumber("Module/Azimuth/kD", DriveConstants.kModuleControllerConfigs.azimuthController().getD());
    public static final LoggedTunableNumber azimuthS = new LoggedTunableNumber("Module/Azimuth/kS", DriveConstants.kModuleControllerConfigs.azimuthFF().getKs());

    // Different set points for the azimuth and drive motors //
    private Double velocitySetpointMPS = null;
    private Rotation2d azimuthSetpointAngle = null;

    // Current state and current position are updated periodically //
    private SwerveModuleState currentState = new SwerveModuleState();
    private SwerveModulePosition currentPosition = new SwerveModulePosition();

    // IO layer along with the autologged inputs that are processed in this file //
    private ModuleIO io;
    private SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

    private String nameKey;

    public Module(String key, ModuleIO instance){
        this.io = (instance);
        nameKey = "Module/" + key;
    }
    
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Drive/"+ nameKey, inputs);

        currentState = new SwerveModuleState(inputs.driveVelocityMPS, inputs.azimuthPosition);
        currentPosition = new SwerveModulePosition(inputs.drivePositionM, inputs.azimuthPosition);

        // Checks if setpoint is existing before setting a demand // 
        if(velocitySetpointMPS != null){
            io.setDriveVelocity(velocitySetpointMPS);     
        }

        if(azimuthSetpointAngle != null){
            io.setAzimuthPosition(azimuthSetpointAngle);
        }

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                io.setDriveGains(driveP.get(), 0.0, driveD.get(), driveS.get(), driveV.get(), driveA.get());
            }, driveP, driveD, driveS, driveV, driveA);

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> {
                io.setAzimuthGains(azimuthP.get(), 0.0, azimuthD.get(), azimuthS.get(), 0.0, 0.0);
            }, azimuthP, azimuthD, azimuthS);
    }

    /**
     * Reset the position of the azimuth encoder
     */
    public void resetAzimuthEncoder(){
        io.resetAzimuthEncoder();
    }

    /**
     * Run characterization for drive motor gains. Dont use kA from Sys ID
     * Bind this to a button and let it run
     * @param inputVolts volts fed into the motor
     */
    public void runLinearCharacterization(double inputVolts) {
        setAzimuthPosition(Rotation2d.fromRotations(0));
        setDriveVelocity(null);
        setDriveVolts(inputVolts);
    }

    /**
     * Run characterization for azimuth motor gains. kA can be used for MOI
     * Bind this to a button and let it run
     * @param inputVolts volts fed into the motor
     */
    public void runCircularCharacterization(double inputVolts, Rotation2d angle) {
        setAzimuthPosition(angle);
        setDriveVelocity(null);
        setDriveVolts(inputVolts);
    }

    /**
     * Set the desired velocity/position respectivly to the drive and azimuth with no acceleration
     * @param state desired state (position + velocity)
     * @return set state
     */
    public SwerveModuleState setSwerveState(SwerveModuleState state){
        setDriveVelocity(state.speedMetersPerSecond);
        setAzimuthPosition(state.angle);
        return new SwerveModuleState(velocitySetpointMPS, azimuthSetpointAngle);
    }

    /**
     * Set the desired position of the azimuth
     * @param state desired state (position + velocity), velocity will not be used
     * @return set state
     */
    public SwerveModuleState setAzimuthPosition(SwerveModuleState state){
        setAzimuthPosition(state.angle);
        setDriveVelocity(null);
        return new SwerveModuleState(0, azimuthSetpointAngle);
    }

    /**
     * Returns the velocity/position of the drive and azimuth respectively
     * @return current state of the swerve module
     */
    public SwerveModuleState getCurrentState(){
        return currentState;
    }

    /**
     * Returnds the distance traveled/position of the drive and azimuth respectively
     * @return current position of the swerve module
     */
    public SwerveModulePosition getCurrentPosition(){
        return currentPosition;
    }

    /**
     * Set velocity demand to the drive motor
     * @param velocityDemand (units : m/s)
     */
    public void setDriveVelocity(Double velocityDemand){
        velocitySetpointMPS = velocityDemand;
    }

    /**
     * Set voltage to the drive motor
     * @param volts (range: 0-12)
     */
    public void setDriveVolts(Double volts){
        io.setDriveVolts(volts);
    }

    /**
     * Set position to the azimuth motor
     * @param positionDemand (units: rotations)
     */
    public void setAzimuthPosition(Rotation2d positionDemand){
        azimuthSetpointAngle = positionDemand;
    }

    /**
     * Set voltage to the azimuth motor
     * @param volts (range: 0-12)
     */
    public void setAzimuthVolts(Double volts){
        io.setAzimuthVolts(volts);
    }

    /**
     * Get the autologged inputs
     * @return inputs
     */
    public SwerveModuleInputsAutoLogged getInputs(){
        return inputs;
    }

    /**
     * Stop swerve module
     */
    public void stop(){
        io.setAzimuthVolts(0);
        io.setDriveVolts(0);

    }

}
