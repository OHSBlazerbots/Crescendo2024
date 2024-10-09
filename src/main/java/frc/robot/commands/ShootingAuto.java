// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import java.time.Duration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class ShootingAuto extends Command{
  /** Example static factory for an autonomous command. */
    // private final DriveSubsystem m_driver;
    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;
    private final Timer timer;
    private final double duration;

  public ShootingAuto(ShooterSubsystem shooter, IntakeSubsystem intake, double time) {
    // m_driver = driver;
    m_shooter = shooter;
    m_intake = intake;
    duration = time;
    // addRequirements(m_shooter);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      // m_driver.ChassisSpeeds(0.25,0.0,0.0);
      timer.reset();
      timer.start();
      System.out.println("initializing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // m_driver.driveCommand(() ->0,() ->1,() ->0);// drive straight at half
      if(timer.get() <= 0.27){
        m_intake.setIntakeSpeed(1);
        m_shooter.setShooterSpeed(0.05);
      }else if(timer.get() <= 1){
        m_intake.setIntakeSpeed(0);  
        m_shooter.setShooterSpeed(-1);
      }else {
        m_intake.setIntakeSpeed(-1);
      }
      System.out.println("excuting");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeSpeed(0);
    m_shooter.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      // end the command if we have run for a specific amount of time
      return timer.get() > duration;

  }
}