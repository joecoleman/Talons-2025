// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.CANSparkMax;

@SuppressWarnings("unused")
public class MAXSwerveModule {
  private final CANSparkMax m_drivingSpark;
  private final CANSparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  @SuppressWarnings("removal")
  private final SparkMaxPIDController m_drivingClosedLoopController;
  @SuppressWarnings("removal")
  private final SparkMaxPIDController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  @SuppressWarnings({ "removal", "deprecation" })
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new CANSparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_drivingClosedLoopController = m_drivingSpark.getPIDController();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();
    m_turningClosedLoopController = m_turningSpark.getPIDController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    // Apply driving configuration
    m_drivingSpark.restoreFactoryDefaults();
    m_drivingSpark.setIdleMode(CANSparkMax.IdleMode.kBrake); // Replace with actual idle mode
    m_drivingSpark.setSmartCurrentLimit(40); // Replace with actual current limit
    m_drivingSpark.burnFlash();

    // Apply turning configuration
    m_turningSpark.restoreFactoryDefaults();
    m_turningSpark.setIdleMode(CANSparkMax.IdleMode.kBrake); // Replace with actual idle mode
    m_turningSpark.setSmartCurrentLimit(20); // Replace with actual current limit
    m_turningSpark.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  @SuppressWarnings("deprecation")
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(
        correctedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningClosedLoopController.setReference(
        correctedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}