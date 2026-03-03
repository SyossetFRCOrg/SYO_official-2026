// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;
import static frc.robot.util.PhoenixUtil.*;

public class ShooterIOTalonFX implements ShooterIO {

        final VoltageOut voltageRequest = new VoltageOut(0);

        final MotionMagicVelocityVoltage motionMagicVoltageRequest = new MotionMagicVelocityVoltage(0);

        private final TalonFX leftTalon;
        private final TalonFX centerTalon;
        private final TalonFX rightTalon;

        private static TalonFXConfiguration leftTalonConfig = new TalonFXConfiguration();
        private static TalonFXConfiguration centerTalonConfig = new TalonFXConfiguration();
        private static TalonFXConfiguration rightTalonConfig = new TalonFXConfiguration();

        private static final LoggedTunableNumber left_kP = new LoggedTunableNumber("Shooter/Gains/left_kP",
                        ShooterConstants.left_kP);
        private static final LoggedTunableNumber left_kD = new LoggedTunableNumber("Shooter/Gains/left_kD",
                        ShooterConstants.left_kD);
        private static final LoggedTunableNumber left_kS = new LoggedTunableNumber("Shooter/Gains/left_kS",
                        ShooterConstants.left_kS);
        private static final LoggedTunableNumber left_kV = new LoggedTunableNumber("Shooter/Gains/left_kV",
                        ShooterConstants.left_kV);
        private static final LoggedTunableNumber left_kA = new LoggedTunableNumber("Shooter/Gains/left_kA",
                        ShooterConstants.left_kA);

        private static final LoggedTunableNumber center_kP = new LoggedTunableNumber("Shooter/Gains/center_kP",
                        ShooterConstants.center_kP);
        private static final LoggedTunableNumber center_kD = new LoggedTunableNumber("Shooter/Gains/center_kD",
                        ShooterConstants.center_kD);
        private static final LoggedTunableNumber center_kS = new LoggedTunableNumber("Shooter/Gains/center_kS",
                        ShooterConstants.center_kS);
        private static final LoggedTunableNumber center_kV = new LoggedTunableNumber("Shooter/Gains/center_kV",
                        ShooterConstants.center_kV);
        private static final LoggedTunableNumber center_kA = new LoggedTunableNumber("Shooter/Gains/center_kA",
                        ShooterConstants.center_kA);

        private static final LoggedTunableNumber right_kP = new LoggedTunableNumber("Shooter/Gains/right_kP",
                        ShooterConstants.right_kP);
        private static final LoggedTunableNumber right_kD = new LoggedTunableNumber("Shooter/Gains/right_kD",
                        ShooterConstants.right_kD);
        private static final LoggedTunableNumber right_kS = new LoggedTunableNumber("Shooter/Gains/right_kS",
                        ShooterConstants.right_kS);
        private static final LoggedTunableNumber right_kV = new LoggedTunableNumber("Shooter/Gains/right_kV",
                        ShooterConstants.right_kV);
        private static final LoggedTunableNumber right_kA = new LoggedTunableNumber("Shooter/Gains/right_kA",
                        ShooterConstants.right_kA);

        private static final LoggedTunableNumber leftMotionMagicAcceleration = new LoggedTunableNumber(
                        "Shooter/leftMaxAcceleration", ShooterConstants.leftMaxAcceleration);
        private static final LoggedTunableNumber leftMotionMagicJerk = new LoggedTunableNumber("Shooter/leftMaxJerk",
                        ShooterConstants.leftMaxJerk);

        private static final LoggedTunableNumber centerMotionMagicAcceleration = new LoggedTunableNumber(
                        "Shooter/centerMaxAcceleration", ShooterConstants.centerMaxAcceleration);
        private static final LoggedTunableNumber centerMotionMagicJerk = new LoggedTunableNumber(
                        "Shooter/centerMaxJerk", ShooterConstants.centerMaxJerk);

        private static final LoggedTunableNumber rightMotionMagicAcceleration = new LoggedTunableNumber(
                        "Shooter/rightMaxAcceleration", ShooterConstants.rightMaxAcceleration);
        private static final LoggedTunableNumber rightMotionMagicJerk = new LoggedTunableNumber("Shooter/rightMaxJerk",
                        ShooterConstants.rightMaxJerk);

        private final StatusSignal<AngularVelocity> leftShooterVelocity;
        private final StatusSignal<Voltage> leftShooterAppliedVolts;
        private final StatusSignal<Current> leftShooterCurrent;
        private final StatusSignal<Current> leftShooterTorqueCurrent;

        private final StatusSignal<AngularVelocity> centerShooterVelocity;
        private final StatusSignal<Voltage> centerShooterAppliedVolts;
        private final StatusSignal<Current> centerShooterCurrent;
        private final StatusSignal<Current> centerShooterTorqueCurrent;

        private final StatusSignal<AngularVelocity> rightShooterVelocity;
        private final StatusSignal<Voltage> rightShooterAppliedVolts;
        private final StatusSignal<Current> rightShooterCurrent;
        private final StatusSignal<Current> rightShooterTorqueCurrent;

        private final Debouncer shooterConnectedDebounce = new Debouncer(0.5);

        /** Creates a new ShooterIOTalonFX. */
        public ShooterIOTalonFX() {
                leftTalon = new TalonFX(ShooterConstants.leftMotorID, ShooterConstants.canbus);
                centerTalon = new TalonFX(ShooterConstants.centerMotorID, ShooterConstants.canbus);
                rightTalon = new TalonFX(ShooterConstants.rightMotorID, ShooterConstants.canbus);

                // set right and left motors to followers
                leftTalon.setControl(new Follower(centerTalon.getDeviceID(), MotorAlignmentValue.Aligned));
                rightTalon.setControl(new Follower(centerTalon.getDeviceID(), MotorAlignmentValue.Aligned));

                leftTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                leftTalonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
                centerTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                centerTalonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
                rightTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                rightTalonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

                leftTalonConfig.Slot0.kA = left_kA.get();
                leftTalonConfig.Slot0.kD = left_kD.get();
                centerTalonConfig.Slot0.kA = center_kA.get();
                centerTalonConfig.Slot0.kD = center_kD.get();
                rightTalonConfig.Slot0.kA = right_kA.get();
                rightTalonConfig.Slot0.kD = right_kD.get();

                leftTalonConfig.Slot0.kP = left_kP.get();
                leftTalonConfig.Slot0.kS = left_kS.get();
                leftTalonConfig.Slot0.kV = left_kV.get();
                centerTalonConfig.Slot0.kP = center_kP.get();
                centerTalonConfig.Slot0.kS = center_kS.get();
                centerTalonConfig.Slot0.kV = center_kV.get();
                rightTalonConfig.Slot0.kP = right_kP.get();
                rightTalonConfig.Slot0.kS = right_kS.get();
                rightTalonConfig.Slot0.kV = right_kV.get();

                leftTalonConfig.MotionMagic.MotionMagicAcceleration = leftMotionMagicAcceleration.get();
                leftTalonConfig.MotionMagic.MotionMagicJerk = leftMotionMagicJerk.get();
                centerTalonConfig.MotionMagic.MotionMagicAcceleration = centerMotionMagicAcceleration.get();
                centerTalonConfig.MotionMagic.MotionMagicJerk = centerMotionMagicJerk.get();
                rightTalonConfig.MotionMagic.MotionMagicAcceleration = rightMotionMagicAcceleration.get();
                rightTalonConfig.MotionMagic.MotionMagicJerk = rightMotionMagicJerk.get();

                leftTalonConfig.CurrentLimits.StatorCurrentLimit = 60;
                leftTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                leftTalonConfig.CurrentLimits.SupplyCurrentLimit = 50;
                leftTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                centerTalonConfig.CurrentLimits.StatorCurrentLimit = 60;
                centerTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                centerTalonConfig.CurrentLimits.SupplyCurrentLimit = 50;
                centerTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                rightTalonConfig.CurrentLimits.StatorCurrentLimit = 60;
                rightTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                rightTalonConfig.CurrentLimits.SupplyCurrentLimit = 50;
                rightTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

                leftTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                centerTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                rightTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                tryUntilOk(5, () -> leftTalon.getConfigurator().apply(leftTalonConfig, 0.25));
                tryUntilOk(5, () -> leftTalon.setPosition(0.0, 0.25));
                tryUntilOk(5, () -> centerTalon.getConfigurator().apply(leftTalonConfig, 0.25));
                tryUntilOk(5, () -> centerTalon.setPosition(0.0, 0.25));
                tryUntilOk(5, () -> rightTalon.getConfigurator().apply(leftTalonConfig, 0.25));
                tryUntilOk(5, () -> rightTalon.setPosition(0.0, 0.25));

                leftShooterVelocity = leftTalon.getVelocity();
                leftShooterAppliedVolts = leftTalon.getMotorVoltage();
                leftShooterCurrent = leftTalon.getSupplyCurrent();
                leftShooterTorqueCurrent = leftTalon.getTorqueCurrent();

                centerShooterVelocity = centerTalon.getVelocity();
                centerShooterAppliedVolts = centerTalon.getMotorVoltage();
                centerShooterCurrent = centerTalon.getSupplyCurrent();
                centerShooterTorqueCurrent = centerTalon.getTorqueCurrent();

                rightShooterVelocity = rightTalon.getVelocity();
                rightShooterAppliedVolts = rightTalon.getMotorVoltage();
                rightShooterCurrent = rightTalon.getSupplyCurrent();
                rightShooterTorqueCurrent = rightTalon.getTorqueCurrent();

                BaseStatusSignal.setUpdateFrequencyForAll(
                                50.0,
                                leftShooterVelocity,
                                leftShooterAppliedVolts,
                                leftShooterCurrent,
                                leftShooterTorqueCurrent,
                                centerShooterVelocity,
                                centerShooterAppliedVolts,
                                centerShooterCurrent,
                                centerShooterTorqueCurrent,
                                rightShooterVelocity,
                                rightShooterAppliedVolts,
                                rightShooterCurrent,
                                rightShooterTorqueCurrent);
                ParentDevice.optimizeBusUtilizationForAll(leftTalon);
                ParentDevice.optimizeBusUtilizationForAll(centerTalon);
                ParentDevice.optimizeBusUtilizationForAll(rightTalon);
        }

        @Override
        public void updateInputs(ShooterIOInputs inputs) {
                LoggedTunableNumber.ifChanged(
                                hashCode(),
                                () -> {
                                        leftTalonConfig.Slot0.kA = left_kA.get();
                                        leftTalonConfig.Slot0.kD = left_kD.get();
                                        // talonConfig.Slot0.kG = kG.get();
                                        leftTalonConfig.Slot0.kP = left_kP.get();
                                        leftTalonConfig.Slot0.kS = left_kS.get();
                                        leftTalonConfig.Slot0.kV = left_kV.get();
                                        tryUntilOk(5, () -> leftTalon.getConfigurator().apply(leftTalonConfig, 0.25));
                                },
                                left_kA,
                                left_kD,
                                // kG,
                                left_kP,
                                left_kS,
                                left_kV);
                LoggedTunableNumber.ifChanged(
                                hashCode(),
                                () -> {
                                        centerTalonConfig.Slot0.kA = center_kA.get();
                                        centerTalonConfig.Slot0.kD = center_kD.get();
                                        // talonConfig.Slot0.kG = kG.get();
                                        centerTalonConfig.Slot0.kP = center_kP.get();
                                        centerTalonConfig.Slot0.kS = center_kS.get();
                                        centerTalonConfig.Slot0.kV = center_kV.get();
                                        tryUntilOk(5, () -> centerTalon.getConfigurator().apply(centerTalonConfig,
                                                        0.25));
                                },
                                center_kA,
                                center_kD,
                                // kG,
                                center_kP,
                                center_kS,
                                center_kV);
                LoggedTunableNumber.ifChanged(
                                hashCode(),
                                () -> {
                                        rightTalonConfig.Slot0.kA = right_kA.get();
                                        rightTalonConfig.Slot0.kD = right_kD.get();
                                        // talonConfig.Slot0.kG = kG.get();
                                        rightTalonConfig.Slot0.kP = right_kP.get();
                                        rightTalonConfig.Slot0.kS = right_kS.get();
                                        rightTalonConfig.Slot0.kV = right_kV.get();
                                        tryUntilOk(5, () -> rightTalon.getConfigurator().apply(rightTalonConfig, 0.25));
                                },
                                right_kA,
                                right_kD,
                                // kG,
                                right_kP,
                                right_kS,
                                right_kV);
                LoggedTunableNumber.ifChanged(
                                hashCode(),
                                () -> {
                                        leftTalonConfig.MotionMagic.MotionMagicAcceleration = leftMotionMagicAcceleration
                                                        .get();
                                        leftTalonConfig.MotionMagic.MotionMagicJerk = leftMotionMagicJerk.get();
                                        tryUntilOk(5, () -> leftTalon.getConfigurator().apply(leftTalonConfig, 0.25));
                                },
                                leftMotionMagicAcceleration,
                                leftMotionMagicJerk);
                var leftTalonStatus = BaseStatusSignal.refreshAll(
                                leftShooterVelocity, leftShooterAppliedVolts, leftShooterCurrent,
                                leftShooterTorqueCurrent);

                inputs.leftConnected = shooterConnectedDebounce.calculate(leftTalonStatus.isOK());

                inputs.leftVelocityRadPerSec = Units
                                .rotationsPerMinuteToRadiansPerSecond(leftShooterVelocity.getValueAsDouble());
                inputs.leftAppliedVolts = leftShooterAppliedVolts.getValueAsDouble();
                inputs.leftCurrentAmps = leftShooterCurrent.getValueAsDouble();

                var rightTalonStatus = BaseStatusSignal.refreshAll(
                                rightShooterVelocity, rightShooterAppliedVolts, rightShooterCurrent,
                                rightShooterTorqueCurrent);

                inputs.rightConnected = shooterConnectedDebounce.calculate(rightTalonStatus.isOK());

                inputs.rightVelocityRadPerSec = Units
                                .rotationsPerMinuteToRadiansPerSecond(rightShooterVelocity.getValueAsDouble());
                inputs.rightAppliedVolts = rightShooterAppliedVolts.getValueAsDouble();
                inputs.rightCurrentAmps = rightShooterCurrent.getValueAsDouble();

                var centerTalonStatus = BaseStatusSignal.refreshAll(
                                centerShooterVelocity, centerShooterAppliedVolts, centerShooterCurrent,
                                centerShooterTorqueCurrent);

                inputs.centerConnected = shooterConnectedDebounce.calculate(centerTalonStatus.isOK());

                inputs.centerVelocityRadPerSec = Units
                                .rotationsPerMinuteToRadiansPerSecond(centerShooterVelocity.getValueAsDouble());
                inputs.centerAppliedVolts = centerShooterAppliedVolts.getValueAsDouble();
                inputs.centerCurrentAmps = centerShooterCurrent.getValueAsDouble();
                // inputs.torqueCurrentAmps = shooterTorqueCurrent.getValueAsDouble();
        }

        public void setVoltage(double velocityRadPerSec) {
                // final VelocityVoltage velocityController = new VelocityVoltage(0);
                // velocityController.Slot = 0;
                // centerTalon.setControl(velocityController.withVelocity(10));
                centerTalon.setControl(voltageRequest.withOutput((velocityRadPerSec)));
        }
        public void setMagicMotionVelocityVoltage(double velocity)
        {
                centerTalon.setControl(motionMagicVoltageRequest.withVelocity(velocity).withAcceleration(centerMotionMagicAcceleration.get()));
                
        }
}
