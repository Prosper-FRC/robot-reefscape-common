package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX kMotor = new TalonFX(IntakeConstants.kMotorID);
  private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
  
  // Motor data we wish to log
  private StatusSignal<AngularVelocity> velocityRotationsPerSec;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Current> statorCurrentAmps;
  private StatusSignal<Temperature> temperatureCelsius;
  
  private VoltageOut voltageControl = new VoltageOut(0.0);

  public IntakeIOTalonFX() {
    // Apply configurations
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = 
        IntakeConstants.kMotorConfiguration.kEnableSupplyCurrentLimit();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = 
        IntakeConstants.kMotorConfiguration.kSupplyCurrentLimitAmps();
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = 
        IntakeConstants.kMotorConfiguration.kEnableStatorCurrentLimit();
    motorConfiguration.CurrentLimits.StatorCurrentLimit = 
        IntakeConstants.kMotorConfiguration.kStatorCurrentLimitAmps();
    motorConfiguration.Voltage.PeakForwardVoltage = 
      IntakeConstants.kMotorConfiguration.kPeakForwardVoltage();
    motorConfiguration.Voltage.PeakReverseVoltage = 
      IntakeConstants.kMotorConfiguration.kPeakReverseVoltage();

    motorConfiguration.MotorOutput.NeutralMode = IntakeConstants.kMotorConfiguration.kNeutralMode();
    motorConfiguration.MotorOutput.Inverted = 
    IntakeConstants.kMotorConfiguration.kInvert() 
        ? InvertedValue.CounterClockwise_Positive 
        : InvertedValue.Clockwise_Positive;

    motorConfiguration.Feedback.SensorToMechanismRatio = IntakeConstants.kGearing;
    // Rotor sensor is the built-in sensor
    motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    kMotor.getConfigurator().apply(motorConfiguration, 1.0);

    // Get status signals from the motor controller
    velocityRotationsPerSec = kMotor.getVelocity();
    appliedVolts = kMotor.getMotorVoltage();
    supplyCurrentAmps = kMotor.getSupplyCurrent();
    statorCurrentAmps = kMotor.getStatorCurrent();
    temperatureCelsius = kMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(IntakeConstants.kStatusSignalUpdateFrequencyHz,
        velocityRotationsPerSec,
        appliedVolts,
        supplyCurrentAmps,
        supplyCurrentAmps,
        statorCurrentAmps,
        temperatureCelsius);

    // Optimize the CANBus utilization by explicitly telling all CAN signals we
    // are not using to simply not be sent over the CANBus
    kMotor.optimizeBusUtilization(0.0, 1.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isMotorConnected = BaseStatusSignal.refreshAll(
      velocityRotationsPerSec,
      appliedVolts,
      supplyCurrentAmps,
      supplyCurrentAmps,
      statorCurrentAmps,
      temperatureCelsius).isOK();

    inputs.velocityRotationsPerSec = velocityRotationsPerSec.getValueAsDouble();
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    kMotor.setControl(voltageControl.withOutput(volts));
  }

   @Override
  public void stop() {
    kMotor.setControl(new NeutralOut());
  }
}
