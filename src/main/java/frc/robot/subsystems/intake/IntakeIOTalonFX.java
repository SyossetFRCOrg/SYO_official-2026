package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;

public class IntakeIOTalonFX implements IntakeIO {

        final VoltageOut VoltageRequest = new VoltageOut(0);

        private final TalonFX rollerTalon;
        private static TalonFXConfiguration rollerTalonConfig = new TalonFXConfiguration();

        private final TalonFX hopperTalon;
        private static TalonFXConfiguration hopperTalonConfig = new TalonFXConfiguration();

        // private final TalonFX armTalon;
        // private static TalonFXConfiguration armTalonConfig = new TalonFXConfiguration();

        private static final LoggedTunableNumber roller_kP = new LoggedTunableNumber("Intake/Gains/roller_kP",
                        IntakeConstants.roller_kP);
        private static final LoggedTunableNumber roller_kD = new LoggedTunableNumber("Intake/Gains/roller_kD",
                        IntakeConstants.roller_kD);
        private static final LoggedTunableNumber roller_kS = new LoggedTunableNumber("Intake/Gains/roller_kS",
                        IntakeConstants.roller_kS);
        // kV is Voltage given per unit of velocity, in this case volts / rad / s
        private static final LoggedTunableNumber roller_kV = new LoggedTunableNumber("Intake/Gains/roller_kV",
                        IntakeConstants.roller_kV);
        // kA is Voltage given per unit of acceleration, volts / rad / s^2
        private static final LoggedTunableNumber roller_kA = new LoggedTunableNumber("Intake/Gains/roller_kA",
                        IntakeConstants.roller_kA);

        // TODO tune these values
        private static final LoggedTunableNumber hopper_kP = new LoggedTunableNumber("Intake/Gains/hopper_kP", IntakeConstants.hopper_kP);
        private static final LoggedTunableNumber hopper_kD = new LoggedTunableNumber("Intake/Gains/hopper_kD", IntakeConstants.hopper_kD);
        private static final LoggedTunableNumber hopper_kS = new LoggedTunableNumber("Intake/Gains/hopper_kS", IntakeConstants.hopper_kS);
        // kV is Voltage given per unit of velocity, in this case volts / rad / s
        private static final LoggedTunableNumber hopper_kV = new LoggedTunableNumber("Intake/Gains/hopper_kV", IntakeConstants.hopper_kV);
        // kA is Voltage given per unit of acceleration, volts / rad / s^2
        private static final LoggedTunableNumber hopper_kA = new LoggedTunableNumber("Intake/Gains/hopper_kA", IntakeConstants.hopper_kA);

        private static final LoggedTunableNumber rollerMotionMagicAcceleration = new LoggedTunableNumber( "Intake/rollerMaxAcceleration", IntakeConstants.rollerMaxAcceleration);
        private static final LoggedTunableNumber rollerMotionMagicJerk = new LoggedTunableNumber("Intake/rollerMaxJerk", IntakeConstants.rollerMaxJerk);

        private static final LoggedTunableNumber hopperCruiseVelocity = new LoggedTunableNumber( "Intake/hopperCruiseVelocity", IntakeConstants.hopperCruiseVelocity);
        private static final LoggedTunableNumber hopperMotionMagicAcceleration = new LoggedTunableNumber( "Intake/hopperMaxAcceleration", IntakeConstants.hopperMaxAcceleration);
        private static final LoggedTunableNumber hopperMotionMagicJerk = new LoggedTunableNumber(
                        "Intake/hopperMaxJerk", IntakeConstants.hopperMaxJerk);

        private final StatusSignal<AngularVelocity> rollerVelocity;
        private final StatusSignal<Voltage> rollerAppliedVolts;

        private final StatusSignal<Angle> hopperPosition;
        private final StatusSignal<AngularVelocity> hopperVelocity;
        private final StatusSignal<Voltage> hopperAppliedVolts;

        private final Debouncer intakeConnectedDebounce = new Debouncer(0.5);

        public IntakeIOTalonFX() {
                // TODO: set up device id for hopper
                rollerTalon = new TalonFX(IntakeConstants.rollerMotorID, IntakeConstants.canbus);
                hopperTalon = new TalonFX(IntakeConstants.hopperMotorID, IntakeConstants.canbus);


                //TODO tune these values and switch over to motion magic
                rollerTalonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
                rollerTalonConfig.Slot0.kA = roller_kA.get();
                rollerTalonConfig.Slot0.kD = roller_kD.get();
                rollerTalonConfig.Slot0.kP = roller_kP.get();
                rollerTalonConfig.Slot0.kS = roller_kS.get();
                rollerTalonConfig.Slot0.kV = roller_kV.get();
                rollerTalonConfig.MotionMagic.MotionMagicAcceleration = rollerMotionMagicAcceleration.get();
                rollerTalonConfig.MotionMagic.MotionMagicJerk = rollerMotionMagicJerk.get();
                rollerTalonConfig.CurrentLimits.StatorCurrentLimit = 60;
                rollerTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                rollerTalonConfig.CurrentLimits.SupplyCurrentLimit = 50;
                rollerTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                rollerTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                

                hopperTalonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
                hopperTalonConfig.Slot0.kA = hopper_kA.get();
                hopperTalonConfig.Slot0.kD = hopper_kD.get();
                hopperTalonConfig.Slot0.kP = hopper_kP.get();
                hopperTalonConfig.Slot0.kS = hopper_kS.get();
                hopperTalonConfig.Slot0.kV = hopper_kV.get();
                hopperTalonConfig.MotionMagic.MotionMagicAcceleration = hopperMotionMagicAcceleration.get();
                hopperTalonConfig.MotionMagic.MotionMagicJerk = hopperMotionMagicJerk.get();
                hopperTalonConfig.CurrentLimits.StatorCurrentLimit = 60;
                hopperTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                hopperTalonConfig.CurrentLimits.SupplyCurrentLimit = 50;
                hopperTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

                intakeRollerTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                hopperTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                // false // fix this, test this. Positive should be upward
                // ? InvertedValue.Clockwise_Positive
                // : InvertedValue.CounterClockwise_Positive;



                tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerTalonConfig, 0.25));
                tryUntilOk(5, () -> hopperTalon.getConfigurator().apply(hopperTalonConfig, 0.25));
                tryUntilOk(5, () -> hopperTalon.setPosition(0));

                rollerVelocity = rollerTalon.getVelocity();
                rollerAppliedVolts = rollerTalon.getMotorVoltage();

                hopperPosition = hopperTalon.getPosition();
                hopperVelocity = hopperTalon.getVelocity();
                hopperAppliedVolts = hopperTalon.getMotorVoltage();

                BaseStatusSignal.setUpdateFrequencyForAll(
                                50.0,
                                rollerVelocity,
                                rollerAppliedVolts,
                                hopperPosition,
                                hopperVelocity,
                                hopperAppliedVolts);
                ParentDevice.optimizeBusUtilizationForAll(rollerTalon, hopperTalon);
        }

        @Override
        public void updateInputs(IntakeIOInputs inputs) {
                LoggedTunableNumber.ifChanged(
                                hashCode(),
                                () -> {
                                        rollerTalonConfig.Slot0.kA = roller_kA.get();
                                        rollerTalonConfig.Slot0.kD = roller_kD.get();
                                        rollerTalonConfig.Slot0.kP = roller_kP.get();
                                        rollerTalonConfig.Slot0.kS = roller_kS.get();
                                        rollerTalonConfig.Slot0.kV = roller_kV.get();

                                        hopperTalonConfig.Slot0.kA = hopper_kA.get();
                                        hopperTalonConfig.Slot0.kD = hopper_kD.get();
                                        hopperTalonConfig.Slot0.kP = hopper_kP.get();
                                        hopperTalonConfig.Slot0.kS = hopper_kS.get();
                                        hopperTalonConfig.Slot0.kV = hopper_kV.get();
                                        tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerTalonConfig, 0.25));
                                        tryUntilOk(5, () -> hopperTalon.getConfigurator().apply(hopperTalonConfig, 0.25));
                                },
                                roller_kA,
                                roller_kD,
                                roller_kP,
                                roller_kS,
                                roller_kV,
                                hopper_kA,
                                hopper_kD,
                                hopper_kP,
                                hopper_kS,
                                hopper_kV);
                LoggedTunableNumber.ifChanged(
                                hashCode(),
                                () -> {
                                        rollerTalonConfig.MotionMagic.MotionMagicAcceleration = rollerMotionMagicAcceleration.get();
                                        rollerTalonConfig.MotionMagic.MotionMagicJerk = rollerMotionMagicJerk.get();
                                        hopperTalonConfig.MotionMagic.MotionMagicCruiseVelocity = hopperCruiseVelocity.get();
                                        hopperTalonConfig.MotionMagic.MotionMagicAcceleration = hopperMotionMagicAcceleration.get();
                                        hopperTalonConfig.MotionMagic.MotionMagicJerk =  hopperMotionMagicJerk.get();
                                        tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerTalonConfig, 0.25));
                                        tryUntilOk(5, () -> hopperTalon.getConfigurator().apply(hopperTalonConfig, 0.25));
                                },
                                rollerMotionMagicAcceleration,
                                rollerMotionMagicJerk,
                                hopperCruiseVelocity,
                                hopperMotionMagicAcceleration,
                                hopperMotionMagicJerk);

                var rollerTalonStatus = BaseStatusSignal.refreshAll(rollerVelocity, rollerAppliedVolts);
                var hopperTalonStatus = BaseStatusSignal.refreshAll(hopperPosition, hopperVelocity,
                                hopperAppliedVolts);

                inputs.rollerConnected = intakeConnectedDebounce.calculate(rollerTalonStatus.isOK());
                inputs.hopperConnected = intakeConnectedDebounce.calculate(hopperTalonStatus.isOK());

                inputs.rollerVelocityRadPerSec = Units
                                .rotationsPerMinuteToRadiansPerSecond(rollerVelocity.getValueAsDouble());
                inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();

                inputs.hopperPosition = Units.rotationsToRadians(hopperPosition.getValueAsDouble());
                inputs.hopperVelocityRadPerSec = Units
                                .rotationsPerMinuteToRadiansPerSecond(hopperVelocity.getValueAsDouble());
                inputs.hopperAppliedVolts = hopperAppliedVolts.getValueAsDouble();

        }

        // TODO this technically just applies a voltage because we are using voltage
        // request, however based on the needs of our robot this will actually function
        // fine as we just need to speed the subsystem up to a good enough speed
        /** Run intake with velocity */
        public void setRollerVoltage(double voltage) {
                rollerTalon.setControl(VoltageRequest.withOutput((voltage)));
        }

        public void moveIntakeHopperToPosition(double positionRadians) {
                final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0);
                hopperTalon.setControl(motionMagicVoltageRequest.withPosition(positionRadians));
        }

        public void setIntakeHopperVoltage(double voltage) {
                hopperTalon.setControl(VoltageRequest.withOutput(voltage));
        }

}
