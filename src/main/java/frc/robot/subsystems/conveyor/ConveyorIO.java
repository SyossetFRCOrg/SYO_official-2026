// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.conveyor;
import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {

  // required fields
  @AutoLog
  public static class ConveyorIOInputs 
  {
      public boolean connected = false;
      public double velocityRadPerSec = 0.0;
      public double appliedVolts = 0.0;
      public double currentAmps = 0.0;
  }

  public default void updateInputs(ConveyorIOInputs inputs) {}
  
  // velocity should be in rad/sec
  public default void setVelocity(double velocity) {}

}
