package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants.SwerveModuleHardwareConfig;
public class ModuleIOKraken implements ModuleIO{

  // Robot Specific info //
  private Rotation2d angleOffset;

  // Hardware //
  private TalonFX azimuth;
  private TalonFX drive;
  private CANcoder cancoder;

  // Status Signals //
  private StatusSignal<AngularVelocity> driveVelocity;
  private StatusSignal<Voltage> driveVoltage;
  private StatusSignal<Angle> drivePosistion;
  private StatusSignal<Current> driveStatorCurrent;
  private StatusSignal<Temperature> driveTempC;

  private StatusSignal<Angle> azimuthPosistion;
  private StatusSignal<Voltage> azimuthVoltage;
  private StatusSignal<Current> azimuthStatorCurrent;
  private StatusSignal<Temperature> azimuthTempC;

  private StatusSignal<Angle> absolutePosistionSignal;

  // Voltage + Posistion Control //
  private VoltageOut azimuthVoltageControl;
  private VoltageOut driveVoltageControl;
  private PositionVoltage azimuthVoltagePosistion;
  private VelocityVoltage driveVelocityControl;


  public ModuleIOKraken(SwerveModuleHardwareConfig modConstants){
    angleOffset = modConstants.offset();

    drive = new TalonFX(modConstants.drivePort(), Constants.kCanbusName);
    
    // Drive motor factory reset +  configs //
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    drive.getConfigurator().apply(driveConfig);
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 80;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 60;
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Voltage.PeakForwardVoltage = 12.0;
    driveConfig.Voltage.PeakReverseVoltage = -12.0;
    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.kDriveGearRatio / DriveConstants.kWheelCircumferenceMeters;
    driveConfig.Slot0.kP = DriveConstants.kModuleControllerConfigs.driveController().getP();
    driveConfig.Slot0.kD = DriveConstants.kModuleControllerConfigs.driveController().getD();
    driveConfig.Slot0.kS = DriveConstants.kModuleControllerConfigs.driveFF().getKs();
    driveConfig.Slot0.kV = DriveConstants.kModuleControllerConfigs.driveFF().getKv();
    driveConfig.Slot0.kA = DriveConstants.kModuleControllerConfigs.driveFF().getKa();
    drive.getConfigurator().apply(driveConfig);
  
    // Drive status signals //
    driveVelocity = drive.getVelocity();
    drivePosistion = drive.getPosition();
    driveVoltage = drive.getMotorVoltage();
    driveStatorCurrent = drive.getStatorCurrent();
    driveTempC = drive.getDeviceTemp();

    // Cancoder factory reset + status signals //
    cancoder = new CANcoder(modConstants.cancoderPort(), Constants.kCanbusName);
    absolutePosistionSignal = cancoder.getAbsolutePosition();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    cancoder.getConfigurator().apply(encoderConfig);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, absolutePosistionSignal);
    cancoder.optimizeBusUtilization();

    // Azimuth Motor factory reset + configs //
    azimuth = new TalonFX(modConstants.azimuthPort(), Constants.kCanbusName);
    TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();
    azimuth.getConfigurator().apply(azimuthConfig);
    azimuthConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    azimuthConfig.CurrentLimits.StatorCurrentLimit = 40;
    azimuthConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    azimuthConfig.CurrentLimits.SupplyCurrentLimit = 30;
    azimuthConfig.MotorOutput.Inverted = (DriveConstants.kInvertAzimuths) ? 
    InvertedValue.Clockwise_Positive : 
    InvertedValue.CounterClockwise_Positive;
    azimuthConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    azimuthConfig.Voltage.PeakForwardVoltage = 12.0;
    azimuthConfig.Voltage.PeakReverseVoltage = -12.0;
    azimuthConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    azimuthConfig.Feedback.SensorToMechanismRatio = DriveConstants.kAzimuthGearRatio;
    azimuthConfig.Slot0.kP = DriveConstants.kModuleControllerConfigs.azimuthController().getP();
    azimuthConfig.Slot0.kD = DriveConstants.kModuleControllerConfigs.azimuthController().getD();
    azimuthConfig.Slot0.kS = DriveConstants.kModuleControllerConfigs.azimuthFF().getKs();
    azimuthConfig.ClosedLoopGeneral.ContinuousWrap = true;
    azimuth.getConfigurator().apply(azimuthConfig);

    // Azimuth status signals //
    azimuthPosistion = azimuth.getPosition();
    azimuthVoltage = azimuth.getMotorVoltage();
    azimuthStatorCurrent = azimuth.getStatorCurrent();
    azimuthTempC = azimuth.getDeviceTemp();

    // Voltage control //
    azimuthVoltageControl = new VoltageOut(0);
    driveVoltageControl = new VoltageOut(0);

    // Closed - Loop control //
    azimuthVoltagePosistion = new PositionVoltage(0);
    driveVelocityControl = new VelocityVoltage(0);

    resetAzimuthEncoder();
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs){
    inputs.driveConnected = BaseStatusSignal.refreshAll(
      driveVelocity,
      driveVoltage,
      driveStatorCurrent,
      driveTempC).isOK();

    inputs.drivePositionM = drivePosistion.getValueAsDouble();
    inputs.driveVelocityMPS = (driveVelocity.getValueAsDouble());
    inputs.driveStatorAmps = new double[] {driveStatorCurrent.getValueAsDouble()};
    inputs.driveTempC = new double[] {driveTempC.getValueAsDouble()};
    inputs.driveMotorVolts = driveVoltage.getValueAsDouble();

    inputs.azimuthConnected = BaseStatusSignal.refreshAll(
      absolutePosistionSignal,
      azimuthPosistion,
      azimuthVoltage,
      azimuthStatorCurrent,
      azimuthTempC).isOK();

    inputs.azimuthPosition = Rotation2d.fromRotations(azimuthPosistion.getValueAsDouble());
    inputs.azimuthAbsolutePosition = Rotation2d.fromRotations(absolutePosistionSignal.getValueAsDouble()).minus((angleOffset));
    inputs.azimuthStatorAmps = new double[] {azimuthStatorCurrent.getValueAsDouble()};
    inputs.azimuthTempC = new double[] {azimuthTempC.getValueAsDouble()};
    inputs.azimuthMotorVolts = azimuthVoltage.getValueAsDouble();
  }

  @Override
  public void setAzimuthVolts(double volts) {
    // Using this control mode allows for voltage to control the speed of the motor //
    azimuth.setControl(azimuthVoltageControl.withOutput(volts));
  }

  @Override
  public void setAzimuthPosition(Rotation2d rotation) {
    // Using this control mode allows for voltage to control the posistion of the motor //
    // Using the withSlot() allows for specific PID and FF gains to be set into the voltage posistion control //
    azimuth.setControl(azimuthVoltagePosistion.withPosition(rotation.getRotations()).withSlot(0));
  }

  @Override
  public void setAzimuthGains(double kP, double kI, double kD, double kS, double kV, double kA){
    Slot0Configs configs = new Slot0Configs();
    // Only gains neccessary for the drive //
    configs.kP = kP;
    configs.kI = kI;
    configs.kD = kD;
    configs.kS = kS;
    configs.kV = kV;
    configs.kA = kA;
    azimuth.getConfigurator().apply(configs);
  }

  @Override
  public void setDriveVolts(double volts) {
    // Using this control mode allows for voltage to control the speed of the motor //
    drive.setControl(driveVoltageControl.withOutput(volts));
  }

  @Override
  public void setDriveVelocity(double velocityMPS) {
    // Using this control mode allows for voltage to control the posistion of the motor //
    // Using the withSlot() allows for specific PID and FF gains to be set into the voltage posistion control //
    drive.setControl(driveVelocityControl.withVelocity(velocityMPS).withSlot(0));
  }

  @Override
  public void setDriveGains(double kP, double kI, double kD, double kS, double kV, double kA){
    Slot0Configs configs = new Slot0Configs();
    // Only gains neccessary for the drive //
    configs.kP = kP;
    configs.kI = kI;
    configs.kD = kD;
    configs.kS = kS;
    configs.kV = kV;
    configs.kA = kA;
    drive.getConfigurator().apply(configs);
  }

  @Override
  public void resetAzimuthEncoder() {
    azimuth.setPosition(
      Rotation2d.fromRotations(
      absolutePosistionSignal.getValueAsDouble()).minus(angleOffset).getRotations(), 
      2.5);
  }
}
