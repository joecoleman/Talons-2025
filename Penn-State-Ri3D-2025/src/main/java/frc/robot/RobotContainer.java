// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOInputsAutoLogged;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
// import frc.robot.subsystems.elevator.elevatorIO.ElevatorIOSparkMaxInputs; // Removed unused import
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final Subsystem Elevator = null;
    private final DriveSubsystem drive;
    private final Climber climber;
    private final Intake intake;
    private final Elevator elevator;
  
    private final double PROCESSOR_HEIGHT = 0;
    private final double SOURCE_HEIGHT = 8.75;
    private final double L1_HEIGHT = 3;
    private final double L2_HEIGHT = 5.5;
    private final double L3_HEIGHT = 21.5;
    private final double L4_HEIGHT = 52.5;
    private final double TOP_ALGAE_HEIGHT = 40;
  
    private final double PROCESSOR_ANGLE = 0;
    private final double SOURCE_ANGLE = 0.15;
    private final double L1_ANGLE = 0.3;
    private final double L2_ANGLE = 0.225;
    private final double L3_ANGLE = 0.225;
    private final double L4_ANGLE = 0.26;
    private final double TOP_ALGAE_ANGLE = 0;
  
    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
  
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Mode");
  
    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
      switch (Constants.currentMode) {
        case REAL:
        climber = new Climber(new ClimberIOSparkMax());
          intake = new Intake(new IntakeIOSparkMax());
          drive = new DriveSubsystem();
          elevator = new Elevator();
    
          break;
  
        case SIM:
          climber = new Climber(new ClimberIO() {
              @Override
              public void updateInputs(ClimberIOInputsAutoLogged inputs) {
                  // Implement the method here
              }
          });
          intake = new Intake(new IntakeIO() {});
          drive = new DriveSubsystem();
          elevator = new Elevator();

            break;
  
        default:
     // Replayed robot, disable IO implementations
          climber = new Climber(new ClimberIO() {

            @Override
            public void updateInputs(ClimberIOInputsAutoLogged inputs) {
                
                throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
            }});
          intake = new Intake(new IntakeIO() {});
          elevator = new Elevator();
              drive = new DriveSubsystem();
              
          break;
      }
    // Add autoChooser to the dashboard
    SmartDashboard.putData((Sendable) autoChooser);
  
      // Configure the button bindings
      configureButtonBindings();
    }
  
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
  
      // Field centric swerve drive
      ((Subsystem) drive).setDefaultCommand(
          new RunCommand(
              () -> drive.drive(
                  driverController.getLeftY() * 0.6,
                  driverController.getLeftX() * 0.6,
                  -driverController.getRightX() * 0.65,
                  true),
              drive));
  
          // Point wheels in x formation to stop
      driverController.x().onTrue(Commands.runOnce(() -> drive.stopWithX(), drive));
  
     
      // Reset gyro
      driverController
          .start()
          .onTrue(
              Commands.runOnce(
                      () ->
                          drive.resetPose(
                              new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                      drive)
                  .ignoringDisable(true));
  
      // Climber command
      Command climbUpCommand =
          new StartEndCommand(() -> climber.setMotorVoltage(1.5), () -> climber.stopMotor(), climber);
      Command climbDownCommand =
          new StartEndCommand(() -> climber.setMotorVoltage(-4), () -> climber.stopMotor(), climber);
      Command climbHoldCommand =
          new StartEndCommand(
              () -> climber.setMotorVoltage(-1.5), () -> climber.stopMotor(), climber);
  
      driverController.povUp().whileTrue(climbUpCommand);
      driverController.povLeft().whileTrue(climbHoldCommand);
      driverController.povDown().whileTrue(climbDownCommand);
  
      // Eject algae
      Command ejectAlgaeCommand =
          new StartEndCommand(
              () -> intake.setAlgaeVoltage(12), () -> intake.setAlgaeVoltage(0), intake);
      driverController.rightBumper().whileTrue(ejectAlgaeCommand);
  
      Command intakeAlgaeCommand =
          new StartEndCommand(
              () -> intake.setAlgaeVoltage(-12), () -> intake.setAlgaeVoltage(0), intake);
      driverController.rightTrigger().whileTrue(intakeAlgaeCommand);
  
      // Intake coral
      Command intakeCoralCommand =
          new StartEndCommand(
              () -> intake.setCoralIntakeVoltage(-6), () -> intake.setCoralIntakeVoltage(0), intake);
      driverController.leftTrigger().whileTrue(intakeCoralCommand);
  
      Command ejectCoralCommand =
          new StartEndCommand(
              () -> intake.setCoralIntakeVoltage(6), () -> intake.setCoralIntakeVoltage(0), intake);
      driverController.leftBumper().whileTrue(ejectCoralCommand);
  
      // Processor state
      Command liftToProcessorCommand =
          new RunCommand(() -> elevator.setPosition(PROCESSOR_HEIGHT), Elevator);
    Command wristToProcessorCommand =
        new RunCommand(() -> intake.wristAngle(PROCESSOR_ANGLE), intake);
    ParallelCommandGroup processorCommandGroup =
        new ParallelCommandGroup(liftToProcessorCommand, wristToProcessorCommand);
    operatorController.povDown().onTrue(processorCommandGroup);

    // Source state
    Command liftToSourceCommand =
        new RunCommand(() -> elevator.setPosition(SOURCE_HEIGHT));
    Command wristToSourceCommand = new RunCommand(() -> intake.wristAngle(SOURCE_ANGLE), intake);
    ParallelCommandGroup sourceCommandGroup =
        new ParallelCommandGroup(liftToSourceCommand, wristToSourceCommand);
    operatorController.povLeft().onTrue(sourceCommandGroup);

    // L1 state
    Command liftToL1Command = new RunCommand(() -> elevator.setPosition(L1_HEIGHT));
    Command wristToL1Command = new RunCommand(() -> intake.wristAngle(L1_ANGLE), intake);
    ParallelCommandGroup l1CommandGroup =
        new ParallelCommandGroup(liftToL1Command, wristToL1Command);
    operatorController.a().onTrue(l1CommandGroup);

    // L2 state
    Command liftToL2Command = new RunCommand(() -> elevator.setPosition(L2_HEIGHT));
        Command wristToL2Command = new RunCommand(() -> intake.wristAngle(L2_ANGLE), intake);
        ParallelCommandGroup l2CommandGroup =
            new ParallelCommandGroup(liftToL2Command, wristToL2Command);
        operatorController.b().onTrue(l2CommandGroup);
    
        // L3 state
        Command liftToL3Command = new RunCommand(() -> elevator.setPosition(L3_HEIGHT));
        Command wristToL3Command = new RunCommand(() -> intake.wristAngle(L3_ANGLE), intake);
        ParallelCommandGroup l3CommandGroup =
            new ParallelCommandGroup(liftToL3Command, wristToL3Command);
        operatorController.y().onTrue(l3CommandGroup);
    
        // L4 state
        Command liftToL4Command = new RunCommand(() -> elevator.setPosition(L4_HEIGHT));
        Command wristToL4Command = new RunCommand(() -> intake.wristAngle(L4_ANGLE), intake);
        ParallelCommandGroup l4CommandGroup =
            new ParallelCommandGroup(liftToL4Command, wristToL4Command);
        operatorController.x().onTrue(l4CommandGroup);
    
        // Top algae state
        Command liftToTopAlgaeCommand =
            new RunCommand(() -> elevator.setPosition(TOP_ALGAE_HEIGHT));
        Command wristToTopAlgaeCommand =
            new RunCommand(() -> intake.wristAngle(TOP_ALGAE_ANGLE), intake);
        ParallelCommandGroup topAlgaeCommandGroup =
            new ParallelCommandGroup(liftToTopAlgaeCommand, wristToTopAlgaeCommand);
        operatorController.povUp().onTrue(topAlgaeCommandGroup);
    
        // Manual lift
        Command manualLift = new RunCommand(() -> elevator.setPosition(operatorController.getLeftY() * 0.25));
            
        // Command manualWrist =
        //     new RunCommand(() -> intake.setWristVoltage(operatorController.getRightY() * 0.25),
        // intake);
        // ParallelCommandGroup manualCommandGroup = new ParallelCommandGroup(manualLift, manualWrist);
        operatorController.start().whileTrue(manualLift);
      }
    
    
    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
  }
}
