// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

   CommandXboxController m_driverController = new CommandXboxController(0);
    CommandXboxController m_CoDriverController = new CommandXboxController(1);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // AbsoluteDrive closedAbsoluteDriveAdv = new AbsoluteDrive(m_DriveSubsystem,
    //   () -> MathUtil.applyDeadband(m_driverController.getLeftY(),
    //       OperatorConstants.LEFT_Y_DEADBAND),
    //   () -> MathUtil.applyDeadband(m_driverController.getLeftX(),
    //         OperatorConstants.LEFT_X_DEADBAND),
    //   () -> MathUtil.applyDeadband(m_driverController.getRightX(),
    //         OperatorConstants.RIGHT_X_DEADBAND),
    //   m_driverController::getYButtonPressed,
    //   m_driverController::getAButtonPressed,
    //   m_driverController::getXButtonPressed,
    //   m_driverController::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation



    
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = m_DriveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRightX(),
        () -> m_driverController.getRightY()); 
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = m_DriveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = m_DriveSubsystem.simDriveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRawAxis(2));

    m_DriveSubsystem.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {  

    m_driverController     
      .rightBumper() 
      .onTrue(Commands.runOnce(() -> m_ShooterSubsystem.setShooterSpeed(-0.2)))
      .onFalse(Commands.runOnce(() -> m_ShooterSubsystem.setShooterSpeed(0)));
    m_driverController
      .y()
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setIntakeSpeed(0.5)))
      .onFalse(Commands.runOnce(() -> m_IntakeSubsystem.setIntakeSpeed(0)));
    m_driverController
      .a()
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setIntakeSpeed(-0.5)))
      .onFalse(Commands.runOnce(() -> m_IntakeSubsystem.setIntakeSpeed(0)));
    m_driverController
      .x()
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setSwivelSpeed(0.5)))
      .onFalse(Commands.runOnce(() -> m_IntakeSubsystem.setSwivelSpeed(0)));
    m_driverController
      .b()
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setSwivelSpeed(-0.5)))
      .onFalse(Commands.runOnce(() -> m_IntakeSubsystem.setSwivelSpeed(0)));
    m_driverController
      .rightTrigger()
      .onTrue(Commands.runOnce(() ->m_IntakeSubsystem.setSwivelPosition(0)));
     m_driverController
      .leftTrigger()
      .onTrue(Commands.runOnce(() ->m_IntakeSubsystem.setSwivelPosition(-1)));
    m_driverController
      .back()
      .onTrue((new InstantCommand(m_DriveSubsystem::zeroGyro)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
    return Autos.exampleAuto(m_DriveSubsystem);
  }
}
