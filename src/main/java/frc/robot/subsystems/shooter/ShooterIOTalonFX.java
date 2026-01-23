// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.shooter.ShooterConstants;
import static frc.robot.util.PhoenixUtil.*;

public class ShooterIOTalonFX implements ShooterIO {

  final VoltageOut VoltageRequest = new VoltageOut(0);

  private final TalonFX talon;
  private static TalonFXConfiguration talonConfig = new TalonFXConfiguration();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Gains/kP", ShooterConstants.kP);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Gains/kD", ShooterConstants.kD);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Gains/kS", ShooterConstants.kS);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/Gains/kV", ShooterConstants.kV);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/Gains/kA", ShooterConstants.kA);

  private static final LoggedTunableNumber motionMagicAcceleration = new LoggedTunableNumber("Shooter/maxAcceleration",
      ShooterConstants.maxAcceleration);
  private static final LoggedTunableNumber motionMagicJerk = new LoggedTunableNumber("Shooter/maxJerk",
      ShooterConstants.maxJerk);

  private final StatusSignal<Angle> shooterPosition;
  private final StatusSignal<AngularVelocity> shooterVelocity;
  private final StatusSignal<Voltage> shooterAppliedVolts;
  private final StatusSignal<Current> shooterCurrent;
  private final StatusSignal<Current> shooterTorqueCurrent;

  private final Debouncer shooterConnectedDebounce = new Debouncer(0.5);

  /** Creates a new ShooterIOTalonFX. */
  public ShooterIOTalonFX() {
    talon = new TalonFX(ShooterConstants.motorID, ShooterConstants.canBus);

    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    talonConfig.Slot0.kA = kA.get();
    talonConfig.Slot0.kD = kD.get();

    talonConfig.Slot0.kP = kP.get();
    talonConfig.Slot0.kS = kS.get();
    talonConfig.Slot0.kV = kV.get();

    talonConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
    talonConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.get();

    talonConfig.CurrentLimits.StatorCurrentLimit = 60;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfig.CurrentLimits.SupplyCurrentLimit = 50;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
    tryUntilOk(5, () -> talon.setPosition(0.0, 0.25));

    shooterPosition = talon.getPosition();
    shooterVelocity = talon.getVelocity();
    shooterAppliedVolts = talon.getMotorVoltage();
    shooterCurrent = talon.getSupplyCurrent();
    shooterTorqueCurrent = talon.getTorqueCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        shooterPosition,
        shooterVelocity,
        shooterAppliedVolts,
        shooterCurrent,
        shooterTorqueCurrent);
    ParentDevice.optimizeBusUtilizationForAll(talon);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          talonConfig.Slot0.kA = kA.get();
          talonConfig.Slot0.kD = kD.get();
          // talonConfig.Slot0.kG = kG.get();
          talonConfig.Slot0.kP = kP.get();
          talonConfig.Slot0.kS = kS.get();
          talonConfig.Slot0.kV = kV.get();
          tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
        },
        kA,
        kD,
        // kG,
        kP,
        kS,
        kV);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          talonConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
          // talonConfig.MotionMagic.MotionMagicCruiseVelocity =
          // motionMagicVelocity.get();
          talonConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.get();
          tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
        },
        motionMagicAcceleration,
        motionMagicJerk);
    var talonStatus = BaseStatusSignal.refreshAll(
        shooterPosition, shooterVelocity, shooterAppliedVolts, shooterCurrent, shooterTorqueCurrent);

    inputs.connected = shooterConnectedDebounce.calculate(talonStatus.isOK());

    inputs.positionRad = Units.rotationsToRadians(shooterPosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(shooterVelocity.getValueAsDouble());
    inputs.appliedVolts = shooterAppliedVolts.getValueAsDouble();
    inputs.currentAmps = shooterCurrent.getValueAsDouble();
    // inputs.torqueCurrentAmps = shooterTorqueCurrent.getValueAsDouble();
  }

  public void setVelocity(double velocityRadPerSec) {
    talon.setControl(VoltageRequest.withOutput((velocityRadPerSec)));
  }


}
