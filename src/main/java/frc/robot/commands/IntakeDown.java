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
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDown extends Command{
    private final IntakeSubsystem m_intake;
    
    public IntakeDown(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }
@Override
  public void initialize() {
      // m_driver.ChassisSpeeds(0.25,0.0,0.0);
      System.out.println("Intake moving down");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // m_driver.driveCommand(() ->0,() ->1,() ->0);// drive straight at half
    m_intake.setSwivelPosition(30);
    System.out.println("excuting");    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      // end the command if we have run for a specific amount of time
      return m_intake.isIntakeDown();

  }
}
