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

        private final TalonFX armTalon;
        private static TalonFXConfiguration armTalonConfig = new TalonFXConfiguration();


        // TODO tune these values
        private static final LoggedTunableNumber roller_kP = new LoggedTunableNumber("Intake/Gains/roller_kP", IntakeConstants.roller_kP);
        private static final LoggedTunableNumber roller_kD = new LoggedTunableNumber("Intake/Gains/roller_kD", IntakeConstants.roller_kD);
        private static final LoggedTunableNumber roller_kS = new LoggedTunableNumber("Intake/Gains/roller_kS", IntakeConstants.roller_kS);
        private static final LoggedTunableNumber roller_kV = new LoggedTunableNumber("Intake/Gains/roller_kV", IntakeConstants.roller_kV);
        private static final LoggedTunableNumber roller_kA = new LoggedTunableNumber("Intake/Gains/roller_kA", IntakeConstants.roller_kA);

        private static final LoggedTunableNumber arm_kP = new LoggedTunableNumber("Intake/Gains/arm_kP", IntakeConstants.arm_kP);
        private static final LoggedTunableNumber arm_kD = new LoggedTunableNumber("Intake/Gains/arm_kD", IntakeConstants.arm_kD);
        private static final LoggedTunableNumber arm_kS = new LoggedTunableNumber("Intake/Gains/arm_kS", IntakeConstants.arm_kS);
        private static final LoggedTunableNumber arm_kV = new LoggedTunableNumber("Intake/Gains/arm_kV", IntakeConstants.arm_kV);
        private static final LoggedTunableNumber arm_kA = new LoggedTunableNumber("Intake/Gains/arm_kA", IntakeConstants.arm_kA);


        private static final LoggedTunableNumber rollerMotionMagicAcceleration = new LoggedTunableNumber( "Intake/rollerMaxAcceleration", IntakeConstants.rollerMaxAcceleration);
        private static final LoggedTunableNumber rollerMotionMagicJerk = new LoggedTunableNumber("Intake/rollerMaxJerk", IntakeConstants.rollerMaxJerk);

        private static final LoggedTunableNumber armMotionMagicCruiseVelocity = new LoggedTunableNumber( "Intake/armCruiseVelocity", IntakeConstants.armMotionMagicCruiseVelocity);
        private static final LoggedTunableNumber armMotionMagicAcceleration = new LoggedTunableNumber( "Intake/armMaxAcceleration", IntakeConstants.armMaxAcceleration);
        private static final LoggedTunableNumber armMotionMagicJerk = new LoggedTunableNumber("Intake/armMaxJerk", IntakeConstants.armMaxJerk);

        

        private final StatusSignal<AngularVelocity> rollerVelocity;
        private final StatusSignal<Voltage> rollerAppliedVolts;


        private final StatusSignal<Angle> armPosition;
        private final StatusSignal<AngularVelocity> armVelocity;
        private final StatusSignal<Voltage> armAppliedVolts;

        private final Debouncer intakeConnectedDebounce = new Debouncer(0.5);

        public IntakeIOTalonFX() {
                rollerTalon = new TalonFX(IntakeConstants.rollerMotorID, IntakeConstants.canbus);
                armTalon = new TalonFX(IntakeConstants.armMotorID, IntakeConstants.canbus);


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
                

                armTalonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
                armTalonConfig.Slot0.kA = arm_kA.get();
                armTalonConfig.Slot0.kD = arm_kD.get();
                armTalonConfig.Slot0.kP = arm_kP.get();
                armTalonConfig.Slot0.kS = arm_kS.get();
                armTalonConfig.Slot0.kV = arm_kV.get();
                armTalonConfig.MotionMagic.MotionMagicCruiseVelocity = armMotionMagicCruiseVelocity.get();
                armTalonConfig.MotionMagic.MotionMagicAcceleration = armMotionMagicAcceleration.get();
                armTalonConfig.MotionMagic.MotionMagicJerk = armMotionMagicJerk.get();
                armTalonConfig.CurrentLimits.StatorCurrentLimit = 60;
                armTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                armTalonConfig.CurrentLimits.SupplyCurrentLimit = 50;
                armTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

                rollerTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                armTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
               



                tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerTalonConfig, 0.25));
                tryUntilOk(5, () -> armTalon.getConfigurator().apply(armTalonConfig, 0.25));
                tryUntilOk(5, () -> armTalon.setPosition(0));

                rollerVelocity = rollerTalon.getVelocity();
                rollerAppliedVolts = rollerTalon.getMotorVoltage();

                armPosition = armTalon.getPosition();
                armVelocity = armTalon.getVelocity();
                armAppliedVolts = armTalon.getMotorVoltage();
                

                BaseStatusSignal.setUpdateFrequencyForAll(
                                50.0,
                                rollerVelocity,
                                rollerAppliedVolts,
                                armPosition,
                                armVelocity,
                                armAppliedVolts);
                ParentDevice.optimizeBusUtilizationForAll(rollerTalon, armTalon);
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
                                        armTalonConfig.Slot0.kA = arm_kA.get();
                                        armTalonConfig.Slot0.kD = arm_kD.get();
                                        armTalonConfig.Slot0.kP = arm_kP.get();
                                        armTalonConfig.Slot0.kS = arm_kS.get();
                                        armTalonConfig.Slot0.kV = arm_kV.get();
                                        tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerTalonConfig, 0.05));
                                        tryUntilOk(5, () -> armTalon.getConfigurator().apply(armTalonConfig, 0.05));
                                },
                                roller_kA,
                                roller_kD,
                                roller_kP,
                                roller_kS,
                                roller_kV);
                LoggedTunableNumber.ifChanged(
                                hashCode(),
                                () -> {
                                        rollerTalonConfig.MotionMagic.MotionMagicAcceleration = rollerMotionMagicAcceleration.get();
                                        rollerTalonConfig.MotionMagic.MotionMagicJerk = rollerMotionMagicJerk.get();
                                        tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerTalonConfig, 0.25));
                                },
                                rollerMotionMagicAcceleration,
                                rollerMotionMagicJerk);

                var rollerTalonStatus = BaseStatusSignal.refreshAll(rollerVelocity, rollerAppliedVolts);
                var armTalonStatus = BaseStatusSignal.refreshAll(armPosition, armVelocity,armAppliedVolts);

                inputs.rollerConnected = intakeConnectedDebounce.calculate(rollerTalonStatus.isOK());
                inputs.armConnected = intakeConnectedDebounce.calculate(armTalonStatus.isOK());

                inputs.rollerVelocityRadPerSec = Units
                                .rotationsPerMinuteToRadiansPerSecond(rollerVelocity.getValueAsDouble());
                inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();

                inputs.armPosition = Units.rotationsToRadians(armPosition.getValueAsDouble());
                inputs.armVelocityRadPerSec = Units
                                .rotationsPerMinuteToRadiansPerSecond(armVelocity.getValueAsDouble());
                inputs.armAppliedVolts = armAppliedVolts.getValueAsDouble();

        }

        public void setRollerVoltage(double voltage) {
                rollerTalon.setControl(VoltageRequest.withOutput((voltage)));
        }

        
        public void setArmVoltage(double voltage) {
                armTalon.setControl(VoltageRequest.withOutput(voltage));
        }

        public void moveArmToPosition(double positionRadians){
                final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(positionRadians);
                armTalon.setControl(motionMagicVoltageRequest.withPosition(positionRadians));
        }

}
