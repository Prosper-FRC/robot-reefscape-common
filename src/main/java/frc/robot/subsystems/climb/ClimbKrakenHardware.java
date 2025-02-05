// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.climb.ClimbConstants.KrakenConfiguration;

import java.util.List;
import org.littletonrobotics.junction.AutoLog;

/**
 * Class that is used to set the control mode of the Kraken and to read telemetry from the motor.
 */
public class ClimbKrakenHardware {
    /** Set of loggable inputs */
    @AutoLog
    public static class PivotInputs {
        // "Inputs" is just a fancy term of data from a source. In this case, we are reading telemetry
        // data from a motor, such as whether it is connected to the CANbus, or what the position of the
        // motor is.

        public boolean leaderMotorConnected = true;
        public boolean followerMotorConnected = true;

        public Rotation2d inteneralPosition = new Rotation2d();
        public double velocityRadsPerSec = 0.0;
        public double[] appliedVolts = new double[] {};
        public double[] supplyCurrentAmps = new double[] {};
        public double[] statorCurrentAmps = new double[] {};
        public double[] temperatureCelsius = new double[] {};
    }

    private final TalonFX kLeadMotor =
        ClimbConstants.kUseCANivore
            ? new TalonFX(ClimbConstants.kLeaderMotorID, ClimbConstants.kCANBusName)
            : new TalonFX(ClimbConstants.kLeaderMotorID);
    private final TalonFX kFollowMotor =
        ClimbConstants.kUseCANivore
            ? new TalonFX(ClimbConstants.kFollowerMotorID, ClimbConstants.kCANBusName)
            : new TalonFX(ClimbConstants.kFollowerMotorID);

    private DutyCycleEncoder throughBoreEncoder;

    private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    // Motor data we wish to log
    private StatusSignal<Angle> internalPositionRotations;
    private StatusSignal<AngularVelocity> velocityRotationsPerSec;
    private List<StatusSignal<Voltage>> appliedVolts;
    private List<StatusSignal<Current>> supplyCurrentAmps;
    private List<StatusSignal<Current>> statorCurrentAmps;
    private List<StatusSignal<Temperature>> temperatureCelsius;

    // Control modes
    private final VoltageOut kVoltageOut = new VoltageOut(0.0);
    private final MotionMagicVoltage kPositionVoltage = new MotionMagicVoltage(0.0);

    public ClimbKrakenHardware(KrakenConfiguration configuration) {
        kFollowMotor.setControl(
            new Follower(
                ClimbConstants.kLeaderMotorID, ClimbConstants.kFollowerMotorOpposeMasterDirection
            )
        );

        // Apply configurations
        motorConfiguration.Slot0.kP = ClimbConstants.kPivotGains.kP();
        motorConfiguration.Slot0.kI = ClimbConstants.kPivotGains.kI();
        motorConfiguration.Slot0.kD = ClimbConstants.kPivotGains.kD();

        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable =
            configuration.kEnableSupplyCurrentLimit();
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.kSupplyCurrentLimitAmps();
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable =
            configuration.kEnableStatorCurrentLimit();
        motorConfiguration.CurrentLimits.StatorCurrentLimit = configuration.kStatorCurrentLimitAmps();
        motorConfiguration.MotorOutput.NeutralMode = configuration.kNeutralMode();
        motorConfiguration.MotorOutput.Inverted =
            configuration.kInverted()
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        motorConfiguration.Voltage.PeakForwardVoltage = configuration.kPeakForwardVoltage();
        motorConfiguration.Voltage.PeakReverseVoltage = configuration.kPeakReverseVoltage();

        // Reset encoder on startup
        kLeadMotor.setPosition(0.0);

        // Setup feedback sensor for position control
        if (ClimbConstants.kUseThroughBore) {
            // NOTE: The DutyCycleEncoder no longer functions as an absolute encoder.  This part of the
            // code will need to be re-written
            /*
            throughBoreEncoder = new DutyCycleEncoder(PivotConstants.kThroughBoreEncoderPort);
            throughBoreEncoder.setDistancePerRotation(PivotConstants.kThroughBoreEncoderRatio);
            // Reset the position of the mechanism (not necessarily the motor) to the position of the
            // through bore encoder, which measures the absolute position of the mechanism.
            kLeadMotor.setPosition(throughBoreEncoder.getAbsolutePosition(), 1.0);
            */
        }
        motorConfiguration.Feedback.SensorToMechanismRatio = ClimbConstants.kGearRatio;
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        kLeadMotor.getConfigurator().apply(motorConfiguration, 1.0);

        // Get status signals from the motor controller
        internalPositionRotations = kLeadMotor.getPosition();
        velocityRotationsPerSec = kLeadMotor.getVelocity();
        appliedVolts = List.of(kLeadMotor.getMotorVoltage(), kFollowMotor.getMotorVoltage());
        supplyCurrentAmps = List.of(kLeadMotor.getSupplyCurrent(), kFollowMotor.getSupplyCurrent());
        statorCurrentAmps = List.of(kLeadMotor.getStatorCurrent(), kFollowMotor.getStatorCurrent());
        temperatureCelsius = List.of(kLeadMotor.getDeviceTemp(), kFollowMotor.getDeviceTemp());

        BaseStatusSignal.setUpdateFrequencyForAll(
            ClimbConstants.kStatusSignalUpdateFrequencyHz,
            internalPositionRotations,
            velocityRotationsPerSec,
            appliedVolts.get(0),
            appliedVolts.get(1),
            supplyCurrentAmps.get(0),
            supplyCurrentAmps.get(1),
            statorCurrentAmps.get(0),
            statorCurrentAmps.get(1),
            temperatureCelsius.get(0),
            temperatureCelsius.get(1)
        );

        // Optimize the CANBus utilization by explicitly telling all CAN signals we
        // are not using to simply not be sent over the CANBus
        kLeadMotor.optimizeBusUtilization(0.0, 1.0);
        kFollowMotor.optimizeBusUtilization(0.0, 1.0);
    }

    /**
    * Write telemetry data from motor to "inputs" object for logging to network tables
    *
    * @param inputs The logged inputs object
    */
    public void updateInputs(PivotInputs inputs) {
        inputs.leaderMotorConnected =
            BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRotationsPerSec,
                appliedVolts.get(0),
                supplyCurrentAmps.get(0),
                statorCurrentAmps.get(0),
                temperatureCelsius.get(0)
            ).isOK();

        inputs.followerMotorConnected =
            BaseStatusSignal.refreshAll(
                  appliedVolts.get(1),
                  supplyCurrentAmps.get(1),
                  statorCurrentAmps.get(1),
                  temperatureCelsius.get(1)
            ).isOK();

      inputs.inteneralPosition =
          Rotation2d.fromRotations(internalPositionRotations.getValueAsDouble());
      inputs.velocityRadsPerSec =
          Units.rotationsToRadians(velocityRotationsPerSec.getValueAsDouble());
      inputs.appliedVolts =
          appliedVolts.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
      inputs.supplyCurrentAmps =
          supplyCurrentAmps.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
      inputs.statorCurrentAmps =
          statorCurrentAmps.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
      inputs.temperatureCelsius =
          temperatureCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    }

    /**
    * Sets the voltage of the motor
    *
    * @param voltage The voltage to set the motor to
    */
    public void setVoltage(double voltage) {
        kLeadMotor.setControl(kVoltageOut.withOutput(voltage));
    }

    /**
    * Sets the desired position of the motor. Runs using internal PID controller
    *
    * @param positionRads The desired position in radians
    * @param feedforwardOutput Feedforward that will also be applied to the control effort
    */
    public void setPosition(double positionRads) {
        kLeadMotor.setControl(
            kPositionVoltage.withPosition(Units.radiansToRotations(positionRads)).withSlot(0)
        );
    }

    /** Sets the motor control to neutral, then switching to the default neutral control mode */
    public void stop() {
        kLeadMotor.setControl(new NeutralOut());
    }

    /**
    * Update the gains of the internal feedback controller and the internal feedforward model
    *
    * @param p The new proportional gain
    * @param i The new integral gain
    * @param d The new derivative gain
    * @param s The new static gain
    * @param v The new voltage gain
    * @param a The new acceleration gain
    * @param g The new gravity gain
    */
    public void setGains(double p, double i, double d, double s, double v, double a, double g) {
        var slotConfiguration = new Slot0Configs();

        slotConfiguration.kP = p;
        slotConfiguration.kI = i;
        slotConfiguration.kD = d;
        slotConfiguration.kS = s;
        slotConfiguration.kV = v;
        slotConfiguration.kA = a;
        slotConfiguration.kG = g;

        kLeadMotor.getConfigurator().apply((slotConfiguration));
    }

    /**
    * Update the gains of the internal trapezoidal motion profile. Units are up to the programmer.
    *
    * @param maxVelocity The new maximum achieveable velocity
    * @param maxAcceleration The new maximum achieveable acceleration
    */
    public void setMotionMagicConstraints(double maxVelocity, double maxAcceleration) {
        var motionMagicConfiguration = new MotionMagicConfigs();

        motionMagicConfiguration.MotionMagicCruiseVelocity = maxVelocity;
        motionMagicConfiguration.MotionMagicAcceleration = maxAcceleration;
        motionMagicConfiguration.MotionMagicJerk = 10.0 * maxAcceleration;

        kLeadMotor.getConfigurator().apply(motionMagicConfiguration);
    }

    /** Reset the relative encoder used for position calculations */
    public void resetEncoder() {
        kLeadMotor.setPosition(0.0);
    }
}
