package frc.robot.subsystems.intake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("CoralWristCurrent", coralWristCurrent);
    table.put("CoralWristVelocity", coralWristVelocity);
    table.put("CoralWristPosition", coralWristPosition);
  }

  @Override
  public void fromLog(LogTable table) {
    coralWristCurrent = table.get("CoralWristCurrent", coralWristCurrent);
    coralWristVelocity = table.get("CoralWristVelocity", coralWristVelocity);
    coralWristPosition = table.get("CoralWristPosition", coralWristPosition);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.coralWristCurrent = this.coralWristCurrent;
    copy.coralWristVelocity = this.coralWristVelocity;
    copy.coralWristPosition = this.coralWristPosition;
    return copy;
  }
}
