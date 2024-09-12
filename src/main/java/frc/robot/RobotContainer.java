package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShootingAuto;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.Constants.IOConstants;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CameraSubsystem;




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
  private final ClimbingSubsystem m_ClimbingSubsystem = new ClimbingSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));


  

  private final CameraSubsystem m_robotCamera = new CameraSubsystem();
  CommandXboxController m_driverController = new CommandXboxController(0);
  CommandXboxController m_CoDriverController = new CommandXboxController(1);

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private Command m_shooterAuto = new ShootingAuto(m_ShooterSubsystem, m_IntakeSubsystem, 3);
  private Command m_DriveAuto = new DriveAuto(m_DriveSubsystem, 1);
  private Command m_IntakeDown = new IntakeDown(m_IntakeSubsystem);
  private Command m_ShootNDriveAuto = m_shooterAuto.andThen(m_DriveAuto);



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
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
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
        () -> m_driverController.getRawAxis(4));

    Command driveFieldOrientedDirectAngleSim = m_DriveSubsystem.simDriveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRawAxis(4));

    m_DriveSubsystem.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
    m_IntakeSubsystem.setDefaultCommand(
      new InstantCommand(m_IntakeSubsystem::writeMetricsToSmartDashboard,m_IntakeSubsystem)
    );

    m_chooser.addOption("Shooting auto", m_shooterAuto);
    m_chooser.addOption("Drive auto", m_DriveAuto);
    m_chooser.addOption("Intake down", m_IntakeDown);
    m_chooser.setDefaultOption("Shoot, then Drive", m_ShootNDriveAuto);
    //m_chooser.setDefaultOption("standby", null);
    SmartDashboard.putData(m_chooser);

  }
  public void resetAll(){
  //   m_ClimbingSubsystem.setClimberSpeed(0);
  //   m_IntakeSubsystem.setIntakeSpeed(0);
  //   m_IntakeSubsystem.setSwivelSpeed(0);
  //   m_ShooterSubsystem.setShooterSpeed(0);
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

     m_CoDriverController     
      .povRight() 
      .onTrue(Commands.runOnce(() -> m_ClimbingSubsystem.setClimberSpeed(1)))
      .onFalse(Commands.runOnce(() -> m_ClimbingSubsystem.setClimberSpeed(0)));
     m_CoDriverController     
      .povLeft() 
      .onTrue(Commands.runOnce(() -> m_ClimbingSubsystem.setClimberSpeed(-1)))
      .onFalse(Commands.runOnce(() -> m_ClimbingSubsystem.setClimberSpeed(0)));
     m_CoDriverController     
      .rightBumper() 
      .onTrue(Commands.runOnce(() -> m_ShooterSubsystem.setShooterSpeed(0)));
    m_CoDriverController     
      .leftBumper() 
      .onTrue(Commands.runOnce(() -> m_ShooterSubsystem.setShooterSpeed(-1)));
    m_CoDriverController
      .y()
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setIntakeSpeed(1)))
      .onFalse(Commands.runOnce(() -> m_IntakeSubsystem.setIntakeSpeed(0)));
    m_CoDriverController
      .a()
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setIntakeSpeed(-1)))
      .onFalse(Commands.runOnce(() -> m_IntakeSubsystem.setIntakeSpeed(0)));
    m_CoDriverController
      .x()
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setSwivelSpeed(0.35)))
      .onFalse(Commands.runOnce(() -> m_IntakeSubsystem.setSwivelSpeed(0)));
    m_CoDriverController
      .b()
      .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.setSwivelSpeed(-0.35)))
      .onFalse(Commands.runOnce(() -> m_IntakeSubsystem.setSwivelSpeed(0)));
    m_CoDriverController
      .axisGreaterThan(1, 0.5)
      .onTrue(Commands.runOnce(() -> m_ClimbingSubsystem.setLeftClimberSpeed(-1)))
      .onFalse(Commands.runOnce(() -> m_ClimbingSubsystem.setLeftClimberSpeed(0)));

     m_CoDriverController
      .axisLessThan(1, -0.5)
      .onTrue(Commands.runOnce(() -> m_ClimbingSubsystem.setLeftClimberSpeed(1)))
      .onFalse(Commands.runOnce(() -> m_ClimbingSubsystem.setLeftClimberSpeed(0)));

    m_CoDriverController
      .axisGreaterThan(5, 0.5)
      .onTrue(Commands.runOnce(() -> m_ClimbingSubsystem.setRightClimberSpeed(-1)))
      .onFalse(Commands.runOnce(() -> m_ClimbingSubsystem.setRightClimberSpeed(0)));

     m_CoDriverController
      .axisLessThan(5, -0.5)
      .onTrue(Commands.runOnce(() -> m_ClimbingSubsystem.setRightClimberSpeed(1)))
      .onFalse(Commands.runOnce(() -> m_ClimbingSubsystem.setRightClimberSpeed(0)));
      
    // m_CoDriverController
    //   .rightTrigger()
    //   .onTrue(Commands.runOnce(() ->m_IntakeSubsystem.setSwivelPosition(0)));
    //  m_CoDriverController
    //   .leftTrigger()
    //   .onTrue(Commands.runOnce(() ->m_IntakeSubsystem.setSwivelPosition(-1)));
    m_driverController
      .back()
      .onTrue((new InstantCommand(m_DriveSubsystem::zeroGyro)));
    m_driverController
      .start()
      .onTrue(new IntakeDown(m_IntakeSubsystem));

  }
  
  /**
   * Use this to pass the autonomous command to tgihe main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return m_chooser.getSelected();
    //return m_shooterAuto;
    // return m_chooser.getSelected();
    // return m_DriveAuto;
    // return m_ShootNDriveAuto;
    return m_IntakeDown;
}
}