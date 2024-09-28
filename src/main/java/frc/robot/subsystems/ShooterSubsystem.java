// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.motorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

    // private static final SparkPIDController m_shooterController = new SparkPIDController(
    //      ShooterConstants.kShooterGains,
    //      ShooterConstants.kShooterMotorPort);
    private CANSparkMax m_leadMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);
    private CANSparkMax m_secondaryMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort2, MotorType.kBrushless);
    private SparkPIDController m_shooterController = m_leadMotor.getPIDController();

    public motorController shooter = new motorController(m_shooterController, 0.15, 0, 0, 0, 1, 1, -1, 5700);


public ShooterSubsystem() {
    m_leadMotor.restoreFactoryDefaults();
    m_secondaryMotor.restoreFactoryDefaults();

     // Current limiting
     /*int TIMEOUT_MS = 10;
     m_Controller.getMotor().configPeakCurrentLimit(20, TIMEOUT_MS); // 20 Amps
     m_shooterController.getMotor().configPeakCurrentDuration(200, TIMEOUT_MS); // 200ms
     m_shooterController.getMotor().configContinuousCurrentLimit(20, TIMEOUT_MS); // 20 Amps
     */
}

public void setShooterSpeed(double speed) {
    // speed = SmartDashboard.getNumber("Shooter/Speed Output", 0);
    System.out.println("speed=" + speed);
    System.out.println("dashboard=" + SmartDashboard.getNumber("Intake/Speed Output", 0));
    m_leadMotor.set(speed);
    m_secondaryMotor.set(-speed);
 }
 @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor set output", m_leadMotor.get());
    SmartDashboard.putNumber("Motor set output", m_secondaryMotor.get());
  }
}

