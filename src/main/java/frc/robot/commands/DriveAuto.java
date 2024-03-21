package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import java.time.Duration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DriveAuto extends CommandBase{
    private final DriveSubsystem m_driver;
    private final Timer timer;
    private final double duration;

    public DriveAuto(DriveSubsystem driver, double time) {
        m_driver = driver;
        duration = time;
        addRequirements(m_driver);
        timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //   m_driver.ChassisSpeeds(0.25,0.0,0.0);
      timer.reset();
      timer.start();
      System.out.println("initializing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //   Command autoDrive = m_driver.driveCommand(() ->-1,() ->0,() ->0, () ->0);
    //   autoDrive.g(m_DriveAuto);
    ChassisSpeeds speedy = m_driver.getTargetSpeeds(-1, 0, 0, 0);
    m_driver.driveFieldOriented(speedy);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      // end the command if we have run for a specific amount of time
      return timer.get() > duration;

  }
}
