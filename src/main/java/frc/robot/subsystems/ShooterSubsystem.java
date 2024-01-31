// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.UpperWenchConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
    private static final PositionPidMotorController m_shooterController = new PositionPidMotorController(
         ShooterConstants.kShooterGains,
         ShooterConstants.kShooterMotorPort);
    private CANSparkMax m_leadMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);
    private CANSparkMax m_secondaryMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);

public shooterSubsystem() {

    m_shooterController.zeroSensors();
    m_shooterController.setPositionZero();
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

    m_leadMotor.set(speed);
    m_secondaryMotor.set(speed);
    writeMetricsToSmartDashboard();

 }

 public void writeMetricsToSmartDashboard() {

    SmartDashboard.putNumber("Motor set output", m_leadMotor.get());
    SmartDashboard.putNumber("Motor set output", m_secondaryMotor.get());
 }

}

