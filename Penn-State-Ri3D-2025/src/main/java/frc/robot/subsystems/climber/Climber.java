// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void setMotorVoltage(double volts) {
    io.setMotorVoltage(volts);
  }

  public void stopMotor() {
    io.stopMotor();
  }

  @SuppressWarnings("static-access")
  public double getMotorAngle() {
    return inputs.motorAngle;
  }

  @SuppressWarnings("static-access")
  public double getMotorCurrent() {
    return inputs.motorCurrent;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", (LoggableInputs) inputs);
  }
}
