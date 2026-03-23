package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {

        final VoltageOut VoltageRequest = new VoltageOut(0);

        private final TalonFX rollerTalon;
        private static TalonFXConfiguration rollerTalonConfig = new TalonFXConfiguration();

        private final TalonFX rightArmTalon;
        private final TalonFX leftArmTalon;
        private static TalonFXConfiguration leftArmTalonConfig = new TalonFXConfiguration();
        private static TalonFXConfiguration rightArmTalonConfig = new TalonFXConfiguration();


        // private static final LoggedTunableNumber roller_kP = new LoggedTunableNumber("Intake/Gains/roller_kP", IntakeConstants.roller_kP);
        // private static final LoggedTunableNumber roller_kD = new LoggedTunableNumber("Intake/Gains/roller_kD", IntakeConstants.roller_kD);
        // private static final LoggedTunableNumber roller_kS = new LoggedTunableNumber("Intake/Gains/roller_kS", IntakeConstants.roller_kS);
        // private static final LoggedTunableNumber roller_kV = new LoggedTunableNumber("Intake/Gains/roller_kV", IntakeConstants.roller_kV);
        // private static final LoggedTunableNumber roller_kA = new LoggedTunableNumber("Intake/Gains/roller_kA", IntakeConstants.roller_kA);

        // private static final LoggedTunableNumber arm_kP = new LoggedTunableNumber("Intake/Gains/arm_kP", IntakeConstants.arm_kP);
        // private static final LoggedTunableNumber arm_kD = new LoggedTunableNumber("Intake/Gains/arm_kD", IntakeConstants.arm_kD);
        // private static final LoggedTunableNumber arm_kS = new LoggedTunableNumber("Intake/Gains/arm_kS", IntakeConstants.arm_kS);
        // private static final LoggedTunableNumber arm_kV = new LoggedTunableNumber("Intake/Gains/arm_kV", IntakeConstants.arm_kV);
        // private static final LoggedTunableNumber arm_kA = new LoggedTunableNumber("Intake/Gains/arm_kA", IntakeConstants.arm_kA);
        // private static final LoggedTunableNumber arm_kG = new LoggedTunableNumber("Intake/Gains/arm_kG", IntakeConstants.arm_kG);


        // private static final LoggedTunableNumber rollerMotionMagicAcceleration = new LoggedTunableNumber( "Intake/rollerMaxAcceleration", IntakeConstants.rollerMaxAcceleration);
        // private static final LoggedTunableNumber rollerMotionMagicJerk = new LoggedTunableNumber("Intake/rollerMaxJerk", IntakeConstants.rollerMaxJerk);

        // private static final LoggedTunableNumber armMotionMagicCruiseVelocity = new LoggedTunableNumber( "Intake/armCruiseVelocity", IntakeConstants.armMotionMagicCruiseVelocity);
        // private static final LoggedTunableNumber armMotionMagicAcceleration = new LoggedTunableNumber( "Intake/armMaxAcceleration", IntakeConstants.armMaxAcceleration);
        // private static final LoggedTunableNumber armMotionMagicJerk = new LoggedTunableNumber("Intake/armMaxJerk", IntakeConstants.armMaxJerk);

        

        private final StatusSignal<AngularVelocity> rollerVelocity;
        private final StatusSignal<Voltage> rollerAppliedVolts;

        private final StatusSignal<Angle> armPosition;
        private final StatusSignal<AngularVelocity> armVelocity;
        private final StatusSignal<Voltage> armAppliedVolts;

        private final Debouncer intakeConnectedDebounce = new Debouncer(0.5);

        public IntakeIOTalonFX() {
                rollerTalon = new TalonFX(IntakeConstants.rollerMotorID, IntakeConstants.canbus);
                rightArmTalon = new TalonFX(IntakeConstants.rightArmMotorID, IntakeConstants.canbus);
                leftArmTalon = new TalonFX(IntakeConstants.leftArmMotorID, IntakeConstants.canbus);

                leftArmTalon.setControl(new Follower(rightArmTalon.getDeviceID(), MotorAlignmentValue.Opposed));


                rollerTalonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
                rollerTalonConfig.Slot0.kA = IntakeConstants.roller_kA;
                rollerTalonConfig.Slot0.kD = IntakeConstants.roller_kD;
                rollerTalonConfig.Slot0.kP = IntakeConstants.roller_kP;
                rollerTalonConfig.Slot0.kS = IntakeConstants.roller_kS;
                rollerTalonConfig.Slot0.kV = IntakeConstants.roller_kV;
                rollerTalonConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.rollerMaxAcceleration;
                rollerTalonConfig.MotionMagic.MotionMagicJerk = IntakeConstants.rollerMaxJerk;
                rollerTalonConfig.CurrentLimits.StatorCurrentLimit = 60;
                rollerTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                rollerTalonConfig.CurrentLimits.SupplyCurrentLimit = 50;
                rollerTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                

                leftArmTalonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
                leftArmTalonConfig.Slot0.kA = IntakeConstants.arm_kA;
                leftArmTalonConfig.Slot0.kD = IntakeConstants.arm_kD;
                leftArmTalonConfig.Slot0.kP = IntakeConstants.arm_kP;
                leftArmTalonConfig.Slot0.kS = IntakeConstants.arm_kS;
                leftArmTalonConfig.Slot0.kV = IntakeConstants.arm_kV;
                leftArmTalonConfig.Slot0.kG = IntakeConstants.arm_kG;
                leftArmTalonConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.armMotionMagicCruiseVelocity;
                leftArmTalonConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.armMaxAcceleration;
                leftArmTalonConfig.MotionMagic.MotionMagicJerk = IntakeConstants.armMaxJerk;
                leftArmTalonConfig.CurrentLimits.StatorCurrentLimit = 120;
                leftArmTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                leftArmTalonConfig.CurrentLimits.SupplyCurrentLimit = 70;
                leftArmTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

                rightArmTalonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
                rightArmTalonConfig.Slot0.kA = IntakeConstants.arm_kA;
                rightArmTalonConfig.Slot0.kD = IntakeConstants.arm_kD;
                rightArmTalonConfig.Slot0.kP = IntakeConstants.arm_kP;
                rightArmTalonConfig.Slot0.kS = IntakeConstants.arm_kS;
                rightArmTalonConfig.Slot0.kV = IntakeConstants.arm_kV;
                rightArmTalonConfig.Slot0.kG = IntakeConstants.arm_kG;
                rightArmTalonConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.armMotionMagicCruiseVelocity;
                rightArmTalonConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.armMaxAcceleration;
                rightArmTalonConfig.MotionMagic.MotionMagicJerk = IntakeConstants.armMaxJerk;
                rightArmTalonConfig.CurrentLimits.StatorCurrentLimit = 120;
                rightArmTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                rightArmTalonConfig.CurrentLimits.SupplyCurrentLimit = 70;
                rightArmTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

                rollerTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                leftArmTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                leftArmTalonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
                leftArmTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                rightArmTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;



                tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerTalonConfig, 0.25));
                tryUntilOk(5, () -> rightArmTalon.getConfigurator().apply(leftArmTalonConfig, 0.25));
                tryUntilOk(5, () -> leftArmTalon.getConfigurator().apply(rightArmTalonConfig, 0.25));                
                tryUntilOk(5, () -> rightArmTalon.setPosition(0));
                tryUntilOk(5, () -> rightArmTalon.setPosition(0));

                rollerVelocity = rollerTalon.getVelocity();
                rollerAppliedVolts = rollerTalon.getMotorVoltage();

                armPosition = rightArmTalon.getPosition();
                armVelocity = rightArmTalon.getVelocity();
                armAppliedVolts = rightArmTalon.getMotorVoltage();
                

                BaseStatusSignal.setUpdateFrequencyForAll(
                                50.0,
                                rollerVelocity,
                                rollerAppliedVolts,
                                armPosition,
                                armVelocity,
                                armAppliedVolts);
                ParentDevice.optimizeBusUtilizationForAll(rollerTalon, rightArmTalon);
        }

        @Override
        public void updateInputs(IntakeIOInputs inputs) {
                // LoggedTunableNumber.ifChanged(
                //                 hashCode(),
                //                 () -> {
                //                         rollerTalonConfig.Slot0.kA = roller_kA.get();
                //                         rollerTalonConfig.Slot0.kD = roller_kD.get();
                //                         rollerTalonConfig.Slot0.kP = roller_kP.get();
                //                         rollerTalonConfig.Slot0.kS = roller_kS.get();
                //                         rollerTalonConfig.Slot0.kV = roller_kV.get();
                //                         leftArmTalonConfig.Slot0.kA = arm_kA.get();
                //                         leftArmTalonConfig.Slot0.kD = arm_kD.get();
                //                         leftArmTalonConfig.Slot0.kP = arm_kP.get();
                //                         leftArmTalonConfig.Slot0.kS = arm_kS.get();
                //                         leftArmTalonConfig.Slot0.kV = arm_kV.get();
                //                         leftArmTalonConfig.Slot0.kG = arm_kG.get();
                //                         tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerTalonConfig, 0.05));
                //                         tryUntilOk(5, () -> rightArmTalon.getConfigurator().apply(leftArmTalonConfig, 0.05));
                //                 },
                //                 roller_kA,
                //                 roller_kD,
                //                 roller_kP,
                //                 roller_kS,
                //                 roller_kV);
                // LoggedTunableNumber.ifChanged(
                //                 hashCode(),
                //                 () -> {
                //                         rollerTalonConfig.MotionMagic.MotionMagicAcceleration = rollerMotionMagicAcceleration.get();
                //                         rollerTalonConfig.MotionMagic.MotionMagicJerk = rollerMotionMagicJerk.get();
                //                         tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerTalonConfig, 0.25));
                //                 },
                //                 rollerMotionMagicAcceleration,
                //                 rollerMotionMagicJerk);

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
                rightArmTalon.setControl(VoltageRequest.withOutput(voltage));
        }

        public void moveArmToPosition(double positionRotations){
                final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(positionRotations);
                // ArmFeedforward feedforward = new ArmFeedforward(arm_kS.get(), arm_kV.get(), arm_kA.get(), arm_kA.get());
                // motionMagicVoltageRequest.FeedForward = feedforward.calculate(armPosition.getValueAsDouble(), 0);
                rightArmTalon.setControl(motionMagicVoltageRequest.withPosition(positionRotations));
        }
        public void setArmEncoderPosition(double positionRotations){
                rightArmTalon.setPosition(positionRotations);

        }

}
