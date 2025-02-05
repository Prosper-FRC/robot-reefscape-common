package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.kMaxAzimuthAngularRadiansPS;
import static frc.robot.subsystems.drive.DriveConstants.kMaxLinearAcceleration;
import static frc.robot.subsystems.drive.DriveConstants.kMaxLinearSpeed;
import static frc.robot.subsystems.drive.DriveConstants.kMaxRotationalAccelerationRadians;
import static frc.robot.subsystems.drive.DriveConstants.kMaxRotationalSpeedRadians;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.*;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.controllers.HeadingController;
import frc.robot.subsystems.drive.controllers.TeleopController;
import frc.robot.utils.debugging.LoggedTunableNumber;
import frc.robot.utils.debugging.SysIDCharacterization;
import frc.robot.utils.swerve.LocalADStarAK;
import frc.robot.utils.swerve.SwerveSetpoint;
import frc.robot.utils.swerve.SwerveSetpointGenerator;
import frc.robot.utils.swerve.SwerveUtils;

public class Drive extends SubsystemBase{

    public static enum DriveState {
        TELEOP,
        AUTO_HEADING,
        AUTON,
        SYS_ID,
        SNIPER_UP,
        SNIPER_DOWN,
        SNIPER_RIGHT,
        SNIPER_LEFT,
        DRIFT_TEST,
        STOP,
        RIGHT_DEG
    }

    private Module[] modules; 
    private GyroIO gyro;
    private GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private ChassisSpeeds autonDesiredSpeeds = new ChassisSpeeds();

    private Rotation2d robotRotation;
    private SwerveDriveOdometry swerveOdometry;
    private SwerveDrivePoseEstimator swervePoseEstimator;
    private Field2d field = new Field2d();

    private TeleopController teleopController = new TeleopController();
    private HeadingController headingController = new HeadingController();

    private RobotConfig robotConfig;
    private SwerveSetpointGenerator generator;
    private SwerveSetpoint previousSetpoint;
    private PIDConstants translationPathplannerConstants = new PIDConstants(3, 0.0, 0.0);
    private PIDConstants rotationPathplannerConstants = new PIDConstants(1.5, 0.0, 0.0);
    private boolean useGenerator = false;

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.kModuleTranslations);

    private static final LoggedTunableNumber kDriftRate = new LoggedTunableNumber("Drive/DriftRate", DriveConstants.kDriftRate);
    
    // Following two only used for drift test:
    private static final LoggedTunableNumber kOmegaSpeed = new LoggedTunableNumber("Drive/Omega Speed", 0.0);
    private static final LoggedTunableNumber kTranslationSpeed = new LoggedTunableNumber("Drive/Translation Speed", 0.0);

    @AutoLogOutput(key="Drive/CurrentState")
    private DriveState driveState = DriveState.TELEOP;

    @AutoLogOutput(key="Drive/HeadingGoal")
    private Rotation2d headingGoal = new Rotation2d();
    
    public Drive(Module[] modules, GyroIO gyro){
        this.modules = modules;
        this.gyro = gyro;
        robotRotation = gyroInputs.yawPosition;

        swerveOdometry = new SwerveDriveOdometry(kinematics, getRobotRotation(), getModulePositions());
        swervePoseEstimator = new SwerveDrivePoseEstimator(kinematics, getRobotRotation(), getModulePositions(), new Pose2d());
        
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch(Exception e) {
            e.printStackTrace();
            System.out.println("HELP ME I CANT CATCH ROBOTCONFIG");
        }

        generator = new SwerveSetpointGenerator(
            robotConfig, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
            kMaxAzimuthAngularRadiansPS // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
        );

        previousSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(0, 0, 0), 
            new SwerveModuleState[] {
                new SwerveModuleState(), new SwerveModuleState(),
                new SwerveModuleState(), new SwerveModuleState()
            }, DriveFeedforwards.zeros(robotConfig.numModules)); 

        AutoBuilder.configure(
            this::getEstimatedPose,
            this::setPose, 
            this::getChassisSpeeds,
            (speeds) -> autonDesiredSpeeds = speeds, 
            new PPHolonomicDriveController(
                translationPathplannerConstants, 
                rotationPathplannerConstants), 
            robotConfig, 
            () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red, 
            this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> Logger.recordOutput(
        "Drive/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> Logger.recordOutput(
        "Drive/Odometry/TrajectorySetpoint", targetPose));

        SmartDashboard.putData(field);

        headingController.setHeadingGoal(() -> headingGoal);
    }  

    @Override
    public void periodic(){
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for(Module module: modules){
            module.periodic();

            if(DriverStation.isDisabled()){
                module.stop();
            }
        }

        if (gyroInputs.connected) {
            robotRotation = gyroInputs.yawPosition;
        } else {
            robotRotation = Rotation2d.fromRadians(
                (swervePoseEstimator.getEstimatedPosition().getRotation().getRadians()
                    + getChassisSpeeds().omegaRadiansPerSecond * 0.02) % 360.0);
        }


        headingController.updateHeadingControllerConfig();

        swervePoseEstimator.update(robotRotation, getModulePositions());
        swerveOdometry.update(robotRotation, getModulePositions());

        field.setRobotPose(getEstimatedPose());


        // The teleop controller takes in the joystick input and converts it to field relative chassis speeds //
        // This is done periodically to constantly grab the inputs from the joysticks //
        ChassisSpeeds teleopSpeeds = teleopController.computeChassisSpeeds((getEstimatedPose().getRotation()), getChassisSpeeds());

        switch (driveState){
            case TELEOP:
                desiredSpeeds = teleopSpeeds;
                break;

            case SYS_ID:
                break;

            case SNIPER_UP:
                desiredSpeeds = new ChassisSpeeds(0.5, 0, 0);
                break;

            case SNIPER_DOWN:
                desiredSpeeds = new ChassisSpeeds(-0.5, 0, 0);
                break;

            case SNIPER_RIGHT:
                desiredSpeeds = new ChassisSpeeds(0, 0.5, 0);
                break;

            case SNIPER_LEFT:
                desiredSpeeds = new ChassisSpeeds(0, -0.5, 0);
                break;

            case DRIFT_TEST:
                desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(kTranslationSpeed.get(), 0, Math.toRadians(kOmegaSpeed.get())), getRobotRotation());
                break;

            case AUTON:
                desiredSpeeds = new ChassisSpeeds(
                    autonDesiredSpeeds.vxMetersPerSecond, 
                    autonDesiredSpeeds.vyMetersPerSecond,
                    autonDesiredSpeeds.omegaRadiansPerSecond);
                break;

            case STOP:
                desiredSpeeds = null;
                for(Module module : modules) {
                    module.stop();
                }
                break;
            
            case RIGHT_DEG:
                headingGoal = Rotation2d.fromDegrees(-90.0);
                desiredSpeeds = new ChassisSpeeds(
                    teleopSpeeds.vxMetersPerSecond, 
                    teleopSpeeds.vyMetersPerSecond,
                    headingController.getSnapOutput(
                        swervePoseEstimator.getEstimatedPosition().getRotation()));
                break;

            default:
                desiredSpeeds = null;
                break;

        }

        Logger.recordOutput("Drive/Odometry/FieldCurrentChassisSpeeds", ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), robotRotation));

        if(desiredSpeeds != null){
            runSwerve(desiredSpeeds);
            Logger.recordOutput("Drive/Odometry/FieldDesiredChassisSpeeds", ChassisSpeeds.fromRobotRelativeSpeeds(getDesiredChassisSpeeds(), robotRotation));
        }

        SmartDashboard.putData(field);
    }  

    public void setDriveEnum(DriveState state){
        driveState = state;
    }

    // Changes the drive state which is logged in AS //
    public Command setDriveStateCommand(DriveState state){
        return Commands.runOnce(() -> setDriveEnum(state), this);
    }

    /**
     * ADD THIS 
     * @param state state to pass in
     * @return command
     */
    public Command setDriveStateCommandContinued(DriveState state){
        return new FunctionalCommand(
            () -> setDriveEnum(state), 
            () -> {}, 
            (interrupted) -> {}, 
            () -> false, 
            this);
    }

    /**
     * Runs characterization to find the gains of the drive motor 
     * @return the command that will be runned
     */
    public Command characterizeDriveMotors() {
        return setDriveStateCommand(DriveState.SYS_ID).andThen(
            SysIDCharacterization.runDriveSysIDTests( (voltage) -> {
                for (var module : modules) module.runLinearCharacterization(voltage);
        }, this));
    }

    public void setChassisSpeeds(ChassisSpeeds speeds){
        desiredSpeeds = speeds;
    }

    public void setVoltage(double voltage){
        for(Module mod : modules){
            mod.setDriveVolts(voltage);
        }
    }

    public void runSwerve(ChassisSpeeds speeds) {
        desiredSpeeds = SwerveUtils.discretize(speeds, kDriftRate.get());

        Logger.recordOutput("Drive/Swerve/Running", true);
        // FOR LOGGING
        // Re-purposed through out execution to compare states
        SwerveModuleState[] unOptimizedSetpointStates = kinematics.toSwerveModuleStates(desiredSpeeds);
        for(int i = 0; i <4; i++) {
            SwerveDriveKinematics.desaturateWheelSpeeds(unOptimizedSetpointStates, kMaxLinearSpeed);
            unOptimizedSetpointStates[i] = new SwerveModuleState(
                    unOptimizedSetpointStates[i].speedMetersPerSecond,
                    Math.abs(previousSetpoint.moduleStates()[i].speedMetersPerSecond / kMaxLinearSpeed) < 0.01 ?
                    modules[i].getCurrentState().angle : unOptimizedSetpointStates[i].angle);
            unOptimizedSetpointStates[i].optimize(modules[i].getCurrentState().angle);
            unOptimizedSetpointStates[i].cosineScale(modules[i].getCurrentState().angle);
        }
        Logger.recordOutput("Drive/Swerve/preOptimizedSetpoints", unOptimizedSetpointStates);

        unOptimizedSetpointStates = kinematics.toSwerveModuleStates(desiredSpeeds);
        for(int i = 0; i <4; i++) {
            unOptimizedSetpointStates[i] = new SwerveModuleState(
                    unOptimizedSetpointStates[i].speedMetersPerSecond,
                    Math.abs(previousSetpoint.moduleStates()[i].speedMetersPerSecond / kMaxLinearSpeed) < 0.01 ?
                    modules[i].getCurrentState().angle : unOptimizedSetpointStates[i].angle);
            unOptimizedSetpointStates[i].optimize(modules[i].getCurrentState().angle);
            unOptimizedSetpointStates[i].cosineScale(modules[i].getCurrentState().angle);
        }
        Logger.recordOutput("Drive/Swerve/saturatedPreOptimizedSetpoints", unOptimizedSetpointStates);
        Logger.recordOutput("Drive/Odometry/preOptimizedChassisSpeeds", kinematics.toChassisSpeeds(unOptimizedSetpointStates));

        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(unOptimizedSetpointStates, kMaxLinearSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxLinearSpeed);

        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

        // SwerveModuleState[] prevSetpointStates = previousSetpoint.moduleStates();

        previousSetpoint = generator.generateSetpoint(
            previousSetpoint, 
            desiredSpeeds, 
            new PathConstraints(
                kMaxLinearSpeed, 
                kMaxLinearAcceleration, 
                kMaxRotationalSpeedRadians, 
                kMaxRotationalAccelerationRadians), 
            0.02);

        Logger.recordOutput("Drive/Odometry/generatedFieldRelativeChassisSpeed", ChassisSpeeds.fromRobotRelativeSpeeds(previousSetpoint.robotRelativeSpeeds(), robotRotation));

        for (int i = 0; i < 4; i++) {
            if(useGenerator) {
                setpointStates[i] = new SwerveModuleState(
                    previousSetpoint.moduleStates()[i].speedMetersPerSecond,
                    Math.abs(previousSetpoint.moduleStates()[i].speedMetersPerSecond / kMaxLinearSpeed) < 0.01 ?
                    modules[i].getCurrentState().angle : previousSetpoint.moduleStates()[i].angle);

                unOptimizedSetpointStates[i] = new SwerveModuleState(setpointStates[i].speedMetersPerSecond, setpointStates[i].angle);

                Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/Acceleration", previousSetpoint.feedforwards().accelerationsMPSSq()[i]);
                Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/Force", previousSetpoint.feedforwards().linearForcesNewtons()[i]);
                Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/Current", previousSetpoint.feedforwards().torqueCurrentsAmps()[i]);

                setpointStates[i].optimize(modules[i].getCurrentState().angle);
                boolean isModuleSpeedOptimized = isSpeedOptimized(unOptimizedSetpointStates[i], setpointStates[i]);
                Logger.recordOutput("Drive/Swerve/Feedforward/"+i+"/optimalInvert", isModuleSpeedOptimized);

                setpointStates[i].cosineScale(modules[i].getCurrentState().angle);

                optimizedSetpointStates[i] = modules[i].setSwerveState(setpointStates[i]);
            } else {
                setpointStates[i] = new SwerveModuleState(
                    setpointStates[i].speedMetersPerSecond,
                    Math.abs(setpointStates[i].speedMetersPerSecond / kMaxLinearSpeed) < 0.01 ?
                    modules[i].getCurrentState().angle : setpointStates[i].angle);

                setpointStates[i].optimize(modules[i].getCurrentState().angle);
                setpointStates[i].cosineScale(modules[i].getCurrentState().angle);
                optimizedSetpointStates[i] = modules[i].setSwerveState(setpointStates[i]);
            }
        }
        
        Logger.recordOutput("Drive/Swerve/Setpoints", unOptimizedSetpointStates);
        Logger.recordOutput("Drive/Swerve/SetpointsOptimized", optimizedSetpointStates);
        Logger.recordOutput("Drive/Swerve/SetpointsChassisSpeeds", kinematics.toChassisSpeeds(optimizedSetpointStates));
        Logger.recordOutput("Drive/Odometry/FieldSetpointChassisSpeed", ChassisSpeeds.fromRobotRelativeSpeeds(
            kinematics.toChassisSpeeds(optimizedSetpointStates), robotRotation));
    }

    public SwerveModuleState[] reduxDrive(Translation2d translation, double rotation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
            new ChassisSpeeds(
                translation.getX(), 
                translation.getY(), 
                rotation));

        Logger.recordOutput("Drive/Swerve/ReduxSetpoints", swerveModuleStates);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxLinearSpeed);

        Logger.recordOutput("Drive/Swerve/SaturatedReduxSetpoints", swerveModuleStates);
        Logger.recordOutput("Drive/Odometry/FieldReduxChassisSpeeds", ChassisSpeeds.fromRobotRelativeSpeeds(
            kinematics.toChassisSpeeds(swerveModuleStates), robotRotation));
            
        return swerveModuleStates;
    }

    public boolean isSpeedOptimized(SwerveModuleState state, SwerveModuleState optimizedState) {
        return state.speedMetersPerSecond != optimizedState.speedMetersPerSecond;
    }

    public void stop(){
        runSwerve(new ChassisSpeeds());
    } 

    // Method should be used when robot is facing forwards //
    public void resetGyro() {
        robotRotation = Constants.kAlliance == Alliance.Blue ? Rotation2d.fromDegrees(0.0) : Rotation2d.fromDegrees(180.0);
        setPose(new Pose2d(getEstimatedPose().getTranslation(), robotRotation));
    }

    public void resetPose(){
        setPose(new Pose2d());
    }

    public void setPose(Pose2d pose){
        robotRotation = pose.getRotation();
        gyro.resetGyro(pose.getRotation());
        swerveOdometry.resetPosition(robotRotation, getModulePositions(), pose);
        swervePoseEstimator.resetPosition(robotRotation, getModulePositions(), pose);
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotRotation")
    public Rotation2d getRobotRotation(){
        return robotRotation;
    }

    @AutoLogOutput(key = "Drive/Odometry/GyroRotation")
    public Rotation2d getGyroRotation(){
        return gyroInputs.yawPosition;
    }

    @AutoLogOutput(key = "Drive/Odometry/PoseEstimate")
    public Pose2d getEstimatedPose(){
        return swervePoseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Drive/Odometry/DrivePose")
    public Pose2d getOdometryPose(){
        return swerveOdometry.getPoseMeters();
    }

    @AutoLogOutput(key = "Drive/Swerve/MeasuredPosistion")
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] posistions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++){
            posistions[i] = modules[i].getCurrentPosition();
        }

        return posistions;
    }

    @AutoLogOutput(key = "Drive/Swerve/MeasuredStates")
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++){
            states[i] = modules[i].getCurrentState();
        }

        return states;
    }

    @AutoLogOutput(key = "Drive/Odometry/CurrentChassisSpeeds")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Drive/Odometry/DesiredChassisSpeeds")
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return desiredSpeeds;
    }

    public void resetAllEncoders(){
        for(int i = 0; i < 4; i++){
            modules[i].resetAzimuthEncoder();
        }
    }

    public void acceptJoystickInputs(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier){
        teleopController.acceptJoystickInputs(xSupplier, ySupplier, thetaSupplier);
    }
}
