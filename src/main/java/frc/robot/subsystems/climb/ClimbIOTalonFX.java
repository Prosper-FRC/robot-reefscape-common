package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.climb.ClimbConstants.ClimbHardware;
import frc.robot.subsystems.climb.ClimbConstants.ClimbMotorConfiguration;

public class ClimbIOTalonFX implements ClimbIO{
    private final TalonFX kMotor;
    private final DutyCycleEncoder kAbsoluteEncoder;
    private final Rotation2d kOffset;

    private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    private StatusSignal<Angle> positionRotations;
    private StatusSignal<Voltage> appliedVolts;
    private StatusSignal<Current> supplyCurrentAmps;
    private StatusSignal<Current> statorCurrentAmps;
    private StatusSignal<Temperature> temperatureCelsius;

    private final VoltageOut kVoltageControl = new VoltageOut(0.0);

    public ClimbIOTalonFX(
        ClimbHardware hardware, 
        ClimbMotorConfiguration configuration){
        
        kMotor = new TalonFX(hardware.motorId());
        kAbsoluteEncoder = new DutyCycleEncoder(hardware.absoluteEncoderPort());
        kOffset = hardware.absoluteEncoderOffset();

        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = configuration.kEnableStatorCurrentLimit();
        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = configuration.kEnableSupplyCurrentLimit();
        motorConfiguration.CurrentLimits.StatorCurrentLimit = configuration.kStatorCurrentLimitAmps();
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.kSupplyCurrentLimitAmps();
        motorConfiguration.MotorOutput.Inverted = (configuration.kInverted() ? (InvertedValue.CounterClockwise_Positive) : (InvertedValue.Clockwise_Positive));
        motorConfiguration.MotorOutput.NeutralMode = configuration.kNeutralMode();
        motorConfiguration.Voltage.PeakForwardVoltage = configuration.kPeakForwardVoltage();
        motorConfiguration.Voltage.PeakForwardVoltage = configuration.kPeakReverseVoltage();

        motorConfiguration.Feedback.SensorToMechanismRatio = hardware.gearing();
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        kMotor.getConfigurator().apply(motorConfiguration);

        positionRotations = kMotor.getPosition();
        appliedVolts = kMotor.getMotorVoltage();
        supplyCurrentAmps = kMotor.getSupplyCurrent();
        statorCurrentAmps = kMotor.getStatorCurrent();
        temperatureCelsius = kMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            20.0,
            positionRotations, 
            appliedVolts, 
            supplyCurrentAmps, 
            statorCurrentAmps, 
            temperatureCelsius);

        kMotor.optimizeBusUtilization();

        resetPosistion();

    }

    @Override
    public void updateInputs(ClimbIOInputs inputs){
        inputs.isMotorConnected = BaseStatusSignal.refreshAll(
            positionRotations, 
            appliedVolts, 
            supplyCurrentAmps, 
            statorCurrentAmps, 
            temperatureCelsius).isOK();
        inputs.absolutePosistion = Rotation2d.fromRotations(kAbsoluteEncoder.get()).minus(kOffset);
        inputs.appliedVoltage = appliedVolts.getValueAsDouble();
        inputs.relativePosistion = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
    }

    @Override
    public void resetPosistion(){
        Rotation2d value = Rotation2d.fromRotations(kAbsoluteEncoder.get()).minus(kOffset);
        kMotor.setPosition(value.getRotations(), 1.0);
    }

    @Override
    public void setVoltage(double volts){
        kMotor.setControl(kVoltageControl.withOutput(volts));
    }

    @Override
    public void setPosistion(Rotation2d posisiton){
        kMotor.setPosition(posisiton.getRotations());
    }

    @Override
    public void stop(){
        kMotor.setControl(kVoltageControl.withOutput(0));
    }

}
