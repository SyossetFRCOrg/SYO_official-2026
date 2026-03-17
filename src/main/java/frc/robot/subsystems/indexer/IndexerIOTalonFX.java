package frc.robot.subsystems.indexer;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.util.LoggedTunableNumber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOTalonFX implements IndexerIO {
    final VoltageOut VoltageRequest = new VoltageOut(0);

    private final TalonFX talon;
    private static TalonFXConfiguration talonConfig = new TalonFXConfiguration();


    // private static final LoggedTunableNumber kP = new LoggedTunableNumber("Indexer/Gains/kP", IndexerConstants.kP);
    // private static final LoggedTunableNumber kD = new LoggedTunableNumber("Indexer/Gains/kD", IndexerConstants.kD);
    // private static final LoggedTunableNumber kS = new LoggedTunableNumber("Indexer/Gains/kS", IndexerConstants.kS);
    // private static final LoggedTunableNumber kV = new LoggedTunableNumber("Indexer/Gains/kV", IndexerConstants.kV);
    // private static final LoggedTunableNumber kA = new LoggedTunableNumber("Indexer/Gains/kA", IndexerConstants.kA);

    // private static final LoggedTunableNumber motionMagicAcceleration = new LoggedTunableNumber(
    //         "Indexer/maxAcceleration",
    //         IndexerConstants.maxAcceleration);
    // private static final LoggedTunableNumber motionMagicJerk = new LoggedTunableNumber("Indexer/maxJerk",
    //         IndexerConstants.maxJerk);

    private final StatusSignal<AngularVelocity> indexerVelocity;
    private final StatusSignal<Voltage> indexerAppliedVolts;
    private final StatusSignal<Current> indexerCurrent;
    private final StatusSignal<Current> indexerTorqueCurrent;

    private final Debouncer indexerConnectedDebounce = new Debouncer(0.5);

    public IndexerIOTalonFX() {
    talon = new TalonFX(IndexerConstants.motorID, IndexerConstants.canbus);

    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    talonConfig.Slot0.kA = IndexerConstants.kA;
    talonConfig.Slot0.kD = IndexerConstants.kD;

    talonConfig.Slot0.kP = IndexerConstants.kP;
    talonConfig.Slot0.kS = IndexerConstants.kS;
    talonConfig.Slot0.kV = IndexerConstants.kV;

    talonConfig.MotionMagic.MotionMagicAcceleration = IndexerConstants.maxAcceleration;
    talonConfig.MotionMagic.MotionMagicJerk = IndexerConstants.maxJerk;

    talonConfig.CurrentLimits.StatorCurrentLimit = 60;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfig.CurrentLimits.SupplyCurrentLimit = 50;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
    tryUntilOk(5, () -> talon.setPosition(0.0, 0.25));

    indexerVelocity = talon.getVelocity();
    indexerAppliedVolts = talon.getMotorVoltage();
    indexerCurrent = talon.getSupplyCurrent();
    indexerTorqueCurrent = talon.getTorqueCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        indexerVelocity,
        indexerAppliedVolts,
        indexerCurrent,
        indexerTorqueCurrent);
    ParentDevice.optimizeBusUtilizationForAll(talon);
  }


  /**
 * Updates the indexer input signals and handles dynamic configuration changes.
 * * <p>This method refreshes sensor data from the Talon FX and pushes any 
 * updated PID or Motion Magic constants from the dashboard to the controller.
 *
 * @param inputs The loggable input container to populate.
 */
  // @Override
  // public void updateInputs(IndexerIOInputs inputs) {
    
  //   LoggedTunableNumber.ifChanged(
  //       hashCode(),
  //       () -> {
  //         talonConfig.Slot0.kA = kA.get();
  //         talonConfig.Slot0.kD = kD.get();
  //         // talonConfig.Slot0.kG = kG.get();
  //         talonConfig.Slot0.kP = kP.get();
  //         talonConfig.Slot0.kS = kS.get();
  //         talonConfig.Slot0.kV = kV.get();
  //         tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
  //       },
  //       kA,
  //       kD,
  //       // kG,
  //       kP,
  //       kS,
  //       kV);
  //   LoggedTunableNumber.ifChanged(
  //       hashCode(),
  //       () -> {
  //         talonConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
  //         // talonConfig.MotionMagic.MotionMagicCruiseVelocity =
  //         // motionMagicVelocity.get();
  //         talonConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.get();
  //         tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
  //       },
  //       motionMagicAcceleration,
  //       motionMagicJerk);
  //   var talonStatus = BaseStatusSignal.refreshAll(
  //       indexerVelocity, indexerAppliedVolts, indexerCurrent, indexerTorqueCurrent);

  //   inputs.connected = indexerConnectedDebounce.calculate(talonStatus.isOK());

  //   inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(indexerVelocity.getValueAsDouble());
  //   inputs.appliedVolts = indexerAppliedVolts.getValueAsDouble();
  //   inputs.currentAmps = indexerCurrent.getValueAsDouble();
  //   // inputs.torqueCurrentAmps = indexerTorqueCurrent.getValueAsDouble();
  // }

  public void setVoltage(double voltage) {
    talon.setControl(VoltageRequest.withOutput((voltage)));
  }



}
