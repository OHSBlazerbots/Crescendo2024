// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.motorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

    // private static final SparkPIDController m_shooterController = new SparkPIDController(
    //      ShooterConstants.kSho"1oterGains,
    //      ShooteIrConstants.kShooterMotorPort);
    private CANSparkMax m_leadIntakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    private CANSparkMax m_secondaryIntakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort2, MotorType.kBrushless);
    private SparkPIDController m_IntakeController = m_leadIntakeMotor.getPIDController();

    public motorController intake = new motorController(m_IntakeController, 0.15, 0, 0, 0, 1, 1, -1, 5700);

    

public IntakeSubsystem() {
    // m_shooterController.zeroSensors();
    // m_shooterController.setPositionZero();
    m_leadIntakeMotor.restoreFactoryDefaults();
    m_secondaryIntakeMotor.restoreFactoryDefaults();

     // Current limiting
     /*int TIMEOUT_MS = 10;
     m_Controller.getMotor().configPeakCurrentLimit(20, TIMEOUT_MS); // 20 Amps
     m_shooterController.getMotor().configPeakCurrentDuration(200, TIMEOUT_MS); // 200ms
     m_shooterController.getMotor().configContinuousCurrentLimit(20, TIMEOUT_MS); // 20 Amps
     */
}

public void setIntakeSpeed(double speed) {
    // speed = SmartDashboard.getNumber("Shooter/Speed Output", 0);
    // System.out.println("speed=" + speed);
    // System.out.println("dashboard=" + SmartDashboard.getNumber("Shooter/Speed Output", 0));
    m_leadIntakeMotor.set(speed);
    writeMetricsToSmartDashboard();
 }
 public void setIntakeArmSpeed(double speed) {
    // speed = SmartDashboard.getNumber("Shooter/Speed Output", 0);
    // System.out.println("speed=" + speed);
    // System.out.println("dashboard=" + SmartDashboard.getNumber("Shooter/Speed Output", 0));
    m_secondaryIntakeMotor.set(speed);
    writeMetricsToSmartDashboard();
 }
 public void writeMetricsToSmartDashboard() {
    intake.writeMetricsToSmartDashboard();
    SmartDashboard.putNumber("Motor set output", m_leadIntakeMotor.get());
    SmartDashboard.putNumber("Motor set output", m_secondaryIntakeMotor.get());
 }

}

