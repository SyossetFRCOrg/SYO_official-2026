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

public class IntakeIOTalonFX implements IntakeIO{

    final VoltageOut VoltageRequest = new VoltageOut(0);

    private final TalonFX intakeTalon;
    private static TalonFXConfiguration intakeTalonConfig = new TalonFXConfiguration();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/Gains/kP", IntakeConstants.kP);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/Gains/kD", IntakeConstants.kD);
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/Gains/kS", IntakeConstants.kS);
    // kV is Voltage given per unit of velocity, in this case volts / rad / s
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/Gains/kV", IntakeConstants.kV);
    // kA is Voltage given per unit of acceleration, volts / rad / s^2
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/Gains/kA", IntakeConstants.kA);

    private static final LoggedTunableNumber motionMagicAcceleration = 
        new LoggedTunableNumber("Intake/maxAcceleration", IntakeConstants.maxAcceleration);
    private static final LoggedTunableNumber motionMagicJerk =
        new LoggedTunableNumber("Intake/maxJerk", IntakeConstants.maxJerk);

    private final StatusSignal<AngularVelocity> intakeVelocity;
    private final StatusSignal<Voltage> intakeAppliedVolts;
    private final StatusSignal<Current> intakeCurrent;
    private final StatusSignal<Current> intakeTorqueCurrent;

    private final Debouncer intakeConnectedDebounce = new Debouncer(0.5);


    public IntakeIOTalonFX()
    {
        // TODO: set up device id
        intakeTalon = new TalonFX(IntakeConstants.motorID, IntakeConstants.canbus);

        intakeTalonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        intakeTalonConfig.Slot0.kA = kA.get();
        intakeTalonConfig.Slot0.kD = kD.get();
        // talonConfig.Slot0.kG = kG.get();
        intakeTalonConfig.Slot0.kP = kP.get();
        intakeTalonConfig.Slot0.kS = kS.get();
        intakeTalonConfig.Slot0.kV = kV.get();

        intakeTalonConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
        // talonConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicVelocity.get();
        intakeTalonConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.get();

        // talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        // talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        intakeTalonConfig.CurrentLimits.StatorCurrentLimit = 60;
        intakeTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeTalonConfig.CurrentLimits.SupplyCurrentLimit = 50;
        intakeTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // false // fix this, test this.  Positive should be upward
        //     ? InvertedValue.Clockwise_Positive
        //     : InvertedValue.CounterClockwise_Positive;

        tryUntilOk(5, () -> intakeTalon.getConfigurator().apply(intakeTalonConfig, 0.25));

        intakeVelocity = intakeTalon.getVelocity();
        intakeAppliedVolts = intakeTalon.getMotorVoltage();
        intakeCurrent = intakeTalon.getSupplyCurrent();
        intakeTorqueCurrent = intakeTalon.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            intakeVelocity,
            intakeAppliedVolts,
            intakeCurrent,
            intakeTorqueCurrent);
        ParentDevice.optimizeBusUtilizationForAll(intakeTalon);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
            intakeTalonConfig.Slot0.kA = kA.get();
            intakeTalonConfig.Slot0.kD = kD.get();
            // talonConfig.Slot0.kG = kG.get();
            intakeTalonConfig.Slot0.kP = kP.get();
            intakeTalonConfig.Slot0.kS = kS.get();
            intakeTalonConfig.Slot0.kV = kV.get();
            tryUntilOk(5, () -> intakeTalon.getConfigurator().apply(intakeTalonConfig, 0.25));
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
            intakeTalonConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
            // talonConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicVelocity.get();
            intakeTalonConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.get();
                tryUntilOk(5, () -> intakeTalon.getConfigurator().apply(intakeTalonConfig, 0.25));
            },
            motionMagicAcceleration,
            motionMagicJerk);
        var talonStatus =
            BaseStatusSignal.refreshAll(
                intakeVelocity, intakeAppliedVolts, intakeCurrent, intakeTorqueCurrent);

        inputs.connected = intakeConnectedDebounce.calculate(talonStatus.isOK());

        inputs.velocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(intakeVelocity.getValueAsDouble());
        inputs.appliedVolts = intakeAppliedVolts.getValueAsDouble();
        inputs.currentAmps = intakeCurrent.getValueAsDouble();
    }    

    /** Run intake with velocity */
    public void setVelocity(double velocityRadPerSec) {
        intakeTalon.setControl(VoltageRequest.withOutput((velocityRadPerSec)));
    }

}
