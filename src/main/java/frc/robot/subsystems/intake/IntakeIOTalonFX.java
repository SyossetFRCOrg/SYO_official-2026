package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;

public class IntakeIOTalonFX implements IntakeIO {

        final VoltageOut VoltageRequest = new VoltageOut(0);

        private final TalonFX intakeRollerTalon;
        private static TalonFXConfiguration intakeRollerTalonConfig = new TalonFXConfiguration();

        private final TalonFX hopperTalon;
        private static TalonFXConfiguration hopperTalonConfig = new TalonFXConfiguration();

        private static final LoggedTunableNumber roller_kP = new LoggedTunableNumber("Intake/Gains/roller_kP", IntakeConstants.roller_kP);
        private static final LoggedTunableNumber roller_kD = new LoggedTunableNumber("Intake/Gains/roller_kD", IntakeConstants.roller_kD);
        private static final LoggedTunableNumber roller_kS = new LoggedTunableNumber("Intake/Gains/roller_kS", IntakeConstants.roller_kS);
        // kV is Voltage given per unit of velocity, in this case volts / rad / s
        private static final LoggedTunableNumber roller_kV = new LoggedTunableNumber("Intake/Gains/roller_kV", IntakeConstants.roller_kV);
        // kA is Voltage given per unit of acceleration, volts / rad / s^2
        private static final LoggedTunableNumber roller_kA = new LoggedTunableNumber("Intake/Gains/roller_kA", IntakeConstants.roller_kA);

        private static final LoggedTunableNumber hopper_kP = new LoggedTunableNumber("Intake/Gains/hopper_kP", IntakeConstants.roller_kP);
        private static final LoggedTunableNumber hopper_kD = new LoggedTunableNumber("Intake/Gains/hopper_kD", IntakeConstants.roller_kD);
        private static final LoggedTunableNumber hopper_kS = new LoggedTunableNumber("Intake/Gains/hopper_kS", IntakeConstants.roller_kS);
        // kV is Voltage given per unit of velocity, in this case volts / rad / s
        private static final LoggedTunableNumber hopper_kV = new LoggedTunableNumber("Intake/Gains/hopper_kV", IntakeConstants.roller_kV);
        // kA is Voltage given per unit of acceleration, volts / rad / s^2
        private static final LoggedTunableNumber hopper_kA = new LoggedTunableNumber("Intake/Gains/hopper_kA", IntakeConstants.roller_kA);


        private static final LoggedTunableNumber rollerMotionMagicAcceleration = new LoggedTunableNumber( "Intake/rollerMaxAcceleration", IntakeConstants.rollerMaxAcceleration);
        private static final LoggedTunableNumber rollerMotionMagicJerk = new LoggedTunableNumber("Intake/rollerMaxJerk", IntakeConstants.rollerMaxJerk);
        private static final LoggedTunableNumber hopperMotionMagicAcceleration = new LoggedTunableNumber( "Intake/hopperMaxAcceleration", IntakeConstants.rollerMaxAcceleration);
        private static final LoggedTunableNumber hopperMotionMagicJerk = new LoggedTunableNumber("Intake/hopperMaxJerk", IntakeConstants.rollerMaxJerk);

        private final StatusSignal<AngularVelocity> intakeRollerVelocity;
        private final StatusSignal<Voltage> intakeRollerAppliedVolts;
        private final StatusSignal<Current> intakeRollerCurrent;
        private final StatusSignal<Current> intakeRollerTorqueCurrent;

        private final Debouncer intakeConnectedDebounce = new Debouncer(0.5);

        public IntakeIOTalonFX() {
                // TODO: set up device id for hopper
                intakeRollerTalon = new TalonFX(IntakeConstants.rollerMotorID, IntakeConstants.canbus);
                hopperTalon = new TalonFX(IntakeConstants.hopperMotorID, IntakeConstants.canbus);

                intakeRollerTalonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
                intakeRollerTalonConfig.Slot0.kA = roller_kA.get();
                intakeRollerTalonConfig.Slot0.kD = roller_kD.get();
                // talonConfig.Slot0.kG = kG.get();
                intakeRollerTalonConfig.Slot0.kP = roller_kP.get();
                intakeRollerTalonConfig.Slot0.kS = roller_kS.get();
                intakeRollerTalonConfig.Slot0.kV = roller_kV.get();

                intakeRollerTalonConfig.MotionMagic.MotionMagicAcceleration = rollerMotionMagicAcceleration.get();
                // talonConfig.MotionMagic.MotionMagicCruiseVelocity =
                // motionMagicVelocity.get();
                intakeRollerTalonConfig.MotionMagic.MotionMagicJerk = rollerMotionMagicJerk.get();

                // talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
                // talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
                intakeRollerTalonConfig.CurrentLimits.StatorCurrentLimit = 60;
                intakeRollerTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                intakeRollerTalonConfig.CurrentLimits.SupplyCurrentLimit = 50;
                intakeRollerTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

                intakeRollerTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                // false // fix this, test this. Positive should be upward
                // ? InvertedValue.Clockwise_Positive
                // : InvertedValue.CounterClockwise_Positive;

                tryUntilOk(5, () -> intakeRollerTalon.getConfigurator().apply(intakeRollerTalonConfig, 0.25));

                intakeRollerVelocity = intakeRollerTalon.getVelocity();
                intakeRollerAppliedVolts = intakeRollerTalon.getMotorVoltage();
                intakeRollerCurrent = intakeRollerTalon.getSupplyCurrent();
                intakeRollerTorqueCurrent = intakeRollerTalon.getTorqueCurrent();

                BaseStatusSignal.setUpdateFrequencyForAll(
                                50.0,
                                intakeRollerVelocity,
                                intakeRollerAppliedVolts,
                                intakeRollerCurrent,
                                intakeRollerTorqueCurrent);
                ParentDevice.optimizeBusUtilizationForAll(intakeRollerTalon);
        }

        @Override
        public void updateInputs(IntakeIOInputs inputs) {
                LoggedTunableNumber.ifChanged(
                                hashCode(),
                                () -> {
                                        intakeRollerTalonConfig.Slot0.kA = roller_kA.get();
                                        intakeRollerTalonConfig.Slot0.kD = roller_kD.get();
                                        // talonConfig.Slot0.kG = kG.get();
                                        intakeRollerTalonConfig.Slot0.kP = roller_kP.get();
                                        intakeRollerTalonConfig.Slot0.kS = roller_kS.get();
                                        intakeRollerTalonConfig.Slot0.kV = roller_kV.get();
                                        tryUntilOk(5, () -> intakeRollerTalon.getConfigurator()
                                                        .apply(intakeRollerTalonConfig, 0.25));
                                },
                                roller_kA,
                                roller_kD,
                                // kG,
                                roller_kP,
                                roller_kS,
                                roller_kV);
                LoggedTunableNumber.ifChanged(
                                hashCode(),
                                () -> {
                                        intakeRollerTalonConfig.MotionMagic.MotionMagicAcceleration = rollerMotionMagicAcceleration
                                                        .get();
                                        // talonConfig.MotionMagic.MotionMagicCruiseVelocity =
                                        // motionMagicVelocity.get();
                                        intakeRollerTalonConfig.MotionMagic.MotionMagicJerk = rollerMotionMagicJerk
                                                        .get();
                                        tryUntilOk(5, () -> intakeRollerTalon.getConfigurator()
                                                        .apply(intakeRollerTalonConfig, 0.25));
                                },
                                rollerMotionMagicAcceleration,
                                rollerMotionMagicJerk);
                var talonStatus = BaseStatusSignal.refreshAll(
                                intakeRollerVelocity, intakeRollerAppliedVolts, intakeRollerCurrent,
                                intakeRollerTorqueCurrent);

                inputs.rollerConnected = intakeConnectedDebounce.calculate(talonStatus.isOK());

                inputs.rollerVelocityRadPerSec = Units
                                .rotationsPerMinuteToRadiansPerSecond(intakeRollerVelocity.getValueAsDouble());
                inputs.rollerAppliedVolts = intakeRollerAppliedVolts.getValueAsDouble();
                inputs.rollerCurrentAmps = intakeRollerCurrent.getValueAsDouble();
        }

        // TODO this technically just applies a voltage because we are using voltage
        // request, however based on the needs of our robot this will actually function
        // fine as we just need to speed the subsystem up to a good enough speed
        /** Run intake with velocity */
        public void setRollerVelocity(double velocityRadPerSec) {
                intakeRollerTalon.setControl(VoltageRequest.withOutput((velocityRadPerSec)));
        }

        public void moveHopperToPosition(double positionRadians) {

        }

}
