package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

// From last year
public class ClimberIOTalonFX implements ClimberIO {

    private final TalonFX talon1, talon2;

    Debouncer climberConnectedDebounce = new Debouncer(.5);

    // private final TalonFX followertalon;
    private static TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    private final StatusSignal<Angle> climberPosition1, climberPosition2;
    private final StatusSignal<AngularVelocity> climberVelocity1, climberVelocity2;
    private final StatusSignal<Voltage> climberAppliedVolts1, climberAppliedVolts2;
    private final StatusSignal<Current> climberCurrent1, climberCurrent2;
    private final StatusSignal<Current> climberTorqueCurrent1, climberTorqueCurrent2;
    private final StatusSignal<Temperature> tempCelsius1, tempCelsius2;

    public ClimberIOTalonFX() {
        talon1 = new TalonFX(ClimberConstants.motor1ID, ClimberConstants.canbus);
        talon2 = new TalonFX(ClimberConstants.motor2ID, ClimberConstants.canbus);


        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        talonConfig.CurrentLimits.StatorCurrentLimit = 120;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimit = 80;
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonConfig.MotorOutput.Inverted =
            false // fix this, test this.  Positive should be upward
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        tryUntilOk(5, () -> talon1.getConfigurator().apply(talonConfig, 0.25));
        tryUntilOk(5, () -> talon2.getConfigurator().apply(talonConfig, 0.25));
        tryUntilOk(5, () -> talon1.setPosition(0.0, 0.25));
        tryUntilOk(5, () -> talon2.setPosition(0.0, 0.25));

        climberPosition1 = talon1.getPosition();
        climberVelocity1 = talon1.getVelocity();
        climberAppliedVolts1 = talon1.getMotorVoltage();
        climberCurrent1 = talon1.getSupplyCurrent();
        climberTorqueCurrent1 = talon1.getTorqueCurrent();
        tempCelsius1 = talon1.getDeviceTemp();

        climberPosition2 = talon2.getPosition();
        climberVelocity2 = talon2.getVelocity();
        climberAppliedVolts2 = talon2.getMotorVoltage();
        climberCurrent2 = talon2.getSupplyCurrent();
        climberTorqueCurrent2 = talon2.getTorqueCurrent();
        tempCelsius2 = talon2.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            climberPosition1,
            climberVelocity1,
            climberAppliedVolts1,
            climberCurrent1,
            climberTorqueCurrent1,
            tempCelsius1,
            climberPosition2,
            climberVelocity2,
            climberAppliedVolts2,
            climberCurrent2,
            climberTorqueCurrent2,
            tempCelsius2);
    
        ParentDevice.optimizeBusUtilizationForAll(talon1, talon2);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {

        var talonStatus1 = BaseStatusSignal.refreshAll(
            climberPosition1,
            climberVelocity1,
            climberAppliedVolts1,
            climberCurrent1,
            climberTorqueCurrent1,
            tempCelsius1
        );

        var talonStatus2 = BaseStatusSignal.refreshAll(
            climberPosition2,
            climberVelocity2,
            climberAppliedVolts2,
            climberCurrent2,
            climberTorqueCurrent2,
            tempCelsius2
        );

        inputs.connectedMotor2 = climberConnectedDebounce.calculate(talonStatus2.isOK());
        inputs.connectedMotor1 = climberConnectedDebounce.calculate(talonStatus1.isOK());

        inputs.positionRad1 = Units.rotationsToRadians(climberPosition1.getValueAsDouble());
        inputs.velocityRadPerSec1 =
            Units.rotationsPerMinuteToRadiansPerSecond(climberVelocity1.getValueAsDouble());
        inputs.appliedVolts1 = climberAppliedVolts1.getValueAsDouble();
        inputs.currentAmps1 = climberTorqueCurrent1.getValueAsDouble();

        inputs.positionRad2 = Units.rotationsToRadians(climberPosition2.getValueAsDouble());
        inputs.velocityRadPerSec2 =
            Units.rotationsPerMinuteToRadiansPerSecond(climberVelocity2.getValueAsDouble());
        inputs.appliedVolts2 = climberAppliedVolts2.getValueAsDouble();
        inputs.currentAmps2 = climberTorqueCurrent2.getValueAsDouble();
    }

    public void stop() {
        talon1.stopMotor();
        talon2.stopMotor();
    }

    /** Resets the angle of the intake to 0. */
    public void resetPosition(double positionRads) {
        talon1.setPosition(Units.radiansToRotations(positionRads));
        talon2.setPosition(Units.radiansToRotations(positionRads));
    }

    /** Run climber with voltage */
    public void setVoltage(double voltage) {
        talon1.setControl(new VoltageOut(voltage));
        talon2.setControl(new VoltageOut(voltage));
    }
}