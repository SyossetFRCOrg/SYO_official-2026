package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;

public class ClimberIOTalonFX implements ClimberIO {

    private static final LoggedTunableNumber motionMagicVelocity =
      new LoggedTunableNumber("Climber/maxVelocity", 100);
    private static final LoggedTunableNumber motionMagicAcceleration =
      new LoggedTunableNumber("Climber/maxAcceleration", 70);
    private static final LoggedTunableNumber motionMagicJerk =
      new LoggedTunableNumber("Climber/maxJerk", 1000);
    
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climber/Gains/kP", 1000);
//  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Climber/Gains/kI", 0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Climber/Gains/kD", 50);
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Climber/Gains/kS", 0);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Climber/Gains/kV", 0);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Climber/Gains/kA", 0);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Climber/Gains/kG", 0);

    private final TalonFX talon;
    private final DynamicMotionMagicTorqueCurrentFOC positionRequest = new DynamicMotionMagicTorqueCurrentFOC(0, motionMagicVelocity.get(), motionMagicJerk.get());

    private static final TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    private final Debouncer climberConnectedDebounce = new Debouncer(.5);

    private double desiredPositionRot = 0;

    private final StatusSignal<Angle> climberPosition;
    private final StatusSignal<AngularVelocity> climberVelocity;
    private final StatusSignal<Voltage> climberAppliedVolts;
    private final StatusSignal<Current> climberCurrent;
    private final StatusSignal<Current> climberTorqueCurrent;

    public ClimberIOTalonFX() {

        talon = new TalonFX(ClimberConstants.motorID, ClimberConstants.canbus);

        // Motor Output
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current Limits
        talonConfig.CurrentLimits.StatorCurrentLimit = 120;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimit = 80;
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 175;
        talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -175;

        // PID
        talonConfig.Slot0.kA = kA.get();
        talonConfig.Slot0.kD = kD.get();
        talonConfig.Slot0.kG = kG.get();
        talonConfig.Slot0.kP = kP.get();
        talonConfig.Slot0.kS = kS.get();
        talonConfig.Slot0.kV = kV.get();

        talonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        // Motion magic
        talonConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicVelocity.get();
        talonConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
        talonConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.get();

        // Status Signals
        climberPosition = talon.getPosition();
        climberVelocity = talon.getVelocity();
        climberAppliedVolts = talon.getMotorVoltage();
        climberCurrent = talon.getSupplyCurrent();
        climberTorqueCurrent = talon.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            climberPosition,
            climberVelocity,
            climberAppliedVolts,
            climberCurrent,
            climberTorqueCurrent);

        ParentDevice.optimizeBusUtilizationForAll(talon);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
            talonConfig.Slot0.kA = kA.get();
            talonConfig.Slot0.kD = kD.get();
            talonConfig.Slot0.kG = kG.get();
            talonConfig.Slot0.kP = kP.get();
            talonConfig.Slot0.kS = kS.get();
            talonConfig.Slot0.kV = kV.get();
            tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
            },
            kA,
            kD,
            kG,
            kP,
            kS,
            kV);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                talonConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
                talonConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicVelocity.get();
                talonConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.get();
                tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
                positionRequest.Velocity = motionMagicVelocity.get();
                positionRequest.Acceleration = motionMagicAcceleration.get();
                positionRequest.Jerk = motionMagicJerk.get();
            },
            motionMagicAcceleration,
            motionMagicJerk,
            motionMagicVelocity);
        

        if (desiredPositionRot > Units.radiansToRotations(climberPosition.getValueAsDouble())) {
            positionRequest.Velocity = motionMagicVelocity.get();
            positionRequest.Acceleration = motionMagicAcceleration.get();
            positionRequest.Jerk = motionMagicJerk.get();

        } else if (desiredPositionRot < Units.radiansToRotations(climberPosition.getValueAsDouble())) {
            positionRequest.Velocity = motionMagicVelocity.get();
            positionRequest.Acceleration = motionMagicAcceleration.get() * .35;
            positionRequest.Jerk = motionMagicJerk.get() * .5;
        }

        var talonStatus =
            BaseStatusSignal.refreshAll(
                climberPosition,
                climberVelocity,
                climberAppliedVolts,
                climberCurrent,
                climberTorqueCurrent
            );


        inputs.motorConnected = climberConnectedDebounce.calculate(talonStatus.isOK());
        inputs.positionRad = Units.rotationsToRadians(climberPosition.getValueAsDouble());
        inputs.velocityRadPerSec = climberVelocity.getValueAsDouble();
        inputs.appliedVolts = climberAppliedVolts.getValueAsDouble();
        inputs.currentAmps = climberCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = climberTorqueCurrent.getValueAsDouble();
    }

    public void setTargetRotations(double rotations) {
        talon.setControl(positionRequest.withPosition(rotations));
    }

    public void stop() {
        talon.stopMotor();
    }
}