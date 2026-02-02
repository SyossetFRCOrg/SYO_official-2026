package frc.robot.subsystems.conveyor;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;

public class ConveyorIOTalonFX implements ConveyorIO {

    private final TalonFX talon;
    private static TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    final VoltageOut VoltageRequest = new VoltageOut(0);

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Conveyor/Gains/kP", ConveyorConstants.kP);
    // private static final LoggedTunableNumber kI = new
    // LoggedTunableNumber("Arm/Gains/kI", 0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Conveyor/Gains/kD", ConveyorConstants.kD);
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Conveyor/Gains/kS", ConveyorConstants.kS);
    // kV is Voltage given per unit of velocity, in this case volts / rad / s
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Conveyor/Gains/kV", ConveyorConstants.kV);
    // kA is Voltage given per unit of acceleration, volts / rad / s^2
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Conveyor/Gains/kA", ConveyorConstants.kA);

    private static final LoggedTunableNumber motionMagicAcceleration = new LoggedTunableNumber(
            "Conveyor/maxAcceleration", ConveyorConstants.maxAcceleration);
    private static final LoggedTunableNumber motionMagicJerk = new LoggedTunableNumber("Conveyor/maxJerk", ConveyorConstants.maxJerk);

    private final StatusSignal<AngularVelocity> conveyorVelocity;
    private final StatusSignal<Voltage> conveyorAppliedVolts;
    private final StatusSignal<Current> conveyorCurrent;
    private final StatusSignal<Current> conveyorTorqueCurrent;

    private final Debouncer conveyorConnectedDebounce = new Debouncer(0.5);

    public ConveyorIOTalonFX() {
        // TODO: device id
        talon = new TalonFX(ConveyorConstants.motorID, ConveyorConstants.canbus);

        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // talonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        talonConfig.Slot0.kA = kA.get();
        talonConfig.Slot0.kD = kD.get();
        // talonConfig.Slot0.kG = kG.get();
        talonConfig.Slot0.kP = kP.get();
        talonConfig.Slot0.kS = kS.get();
        talonConfig.Slot0.kV = kV.get();

        talonConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
        // talonConfig.MotionMagic.MotionMagicCruiseVelocity =
        // motionMagicVelocity.get();
        talonConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.get();

        // talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        // talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        talonConfig.CurrentLimits.StatorCurrentLimit = 60;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimit = 50;
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // false // fix this, test this. Positive should be upward
        // ? InvertedValue.Clockwise_Positive
        // : InvertedValue.CounterClockwise_Positive;

        tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
        tryUntilOk(5, () -> talon.setPosition(0.0, 0.25));

        conveyorVelocity = talon.getVelocity();
        conveyorAppliedVolts = talon.getMotorVoltage();
        conveyorCurrent = talon.getSupplyCurrent();
        conveyorTorqueCurrent = talon.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                conveyorVelocity,
                conveyorAppliedVolts,
                conveyorCurrent,
                conveyorTorqueCurrent);
        ParentDevice.optimizeBusUtilizationForAll(talon);
    }

    @Override
    public void updateInputs(ConveyorIOInputs inputs) {
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
                conveyorVelocity, conveyorAppliedVolts, conveyorCurrent, conveyorTorqueCurrent);

        inputs.connected = conveyorConnectedDebounce.calculate(talonStatus.isOK());

        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(conveyorVelocity.getValueAsDouble());
        inputs.appliedVolts = conveyorAppliedVolts.getValueAsDouble();
        inputs.currentAmps = conveyorCurrent.getValueAsDouble();
        // inputs.torqueCurrentAmps = conveyorTorqueCurrent.getValueAsDouble();
    }

    // @Override
    // public void stop() {
    // talon.stopMotor();
    // }

    /** Resets the angle of the conveyor to 0. */
    public void set(double positionRads) {
        talon.setPosition(Units.radiansToRotations(positionRads));
    }

    /** Run conveyor with velocity */
    public void setVelocity(double velocityRadPerSec) {
        talon.setControl(VoltageRequest.withOutput((velocityRadPerSec)));
        // talon.setControl(torquerequest.withVelocity(velocityRadPerSec));
    }

}
