package frc.robot.subsystems.elevator;

public interface elevatorIO {

    void set(double voltage);

    void stop();

    void setPosition(double position);

    void updateInputs(ElevatorIOInputsAutoLogged inputs);

    double getPosition();

    double getVelocity();

    void resetPosition();

}
