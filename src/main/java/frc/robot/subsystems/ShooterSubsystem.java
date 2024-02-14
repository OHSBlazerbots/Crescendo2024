// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.motor_controller;
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

    double kP = 0.15; 
    double kI = 0.0;
    double kD = 0.0; 
    double kIz = 0.0; 
    double kFF = 1.0; 
    double kMaxOutput = 1.0; 
    double kMinOutput = -1.0;
    double maxRPM = 5700.0;
    

public ShooterSubsystem() {

    // set PID coefficients
    m_shooterController.setP(kP);
    m_shooterController.setI(kI);
    m_shooterController.setD(kD);
    m_shooterController.setIZone(kIz);
    m_shooterController.setFF(kFF);
    m_shooterController.setOutputRange(kMinOutput, kMaxOutput);

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    if((p != kP)) { m_shooterController.setP(p); kP = p; }
    if((i != kI)) { m_shooterController.setI(i); kI = i; }
    if((d != kD)) { m_shooterController.setD(d); kD = d; }
    if((iz != kIz)) { m_shooterController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_shooterController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_shooterController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    
    }


    // m_shooterController.zeroSensors();
    // m_shooterController.setPositionZero();
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
    System.out.println("dashboard=" + SmartDashboard.getNumber("Shooter/Speed Output", 0));
    m_leadMotor.set(speed);
    m_secondaryMotor.set(-speed);
    writeMetricsToSmartDashboard();
 }

 public void writeMetricsToSmartDashboard() {

    SmartDashboard.putNumber("Motor set output", m_leadMotor.get());
    SmartDashboard.putNumber("Motor set output", m_secondaryMotor.get());
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
 }

}

