// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.motorController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

    // private static final SparkPIDController m_shooterController = new SparkPIDController(
    //      ShooterConstants.kSho"1oterGains,
    //      ShooteIrConstants.kShooterMotorPort);
    private CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    private CANSparkMax m_swivelMotor = new CANSparkMax(IntakeConstants.kSwivelMotorPort, MotorType.kBrushless);
    private  DigitalInput m_IntakeLimitSwitch = new DigitalInput(0);
    private RelativeEncoder m_swivelEncoder = m_swivelMotor.getEncoder();
    private SparkPIDController m_IntakeController = m_intakeMotor.getPIDController();
    private SparkPIDController m_SwivelController = m_swivelMotor.getPIDController();
    

public IntakeSubsystem() {
    m_intakeMotor.restoreFactoryDefaults();
    m_swivelMotor.restoreFactoryDefaults();
    m_swivelMotor.setIdleMode(IdleMode.kBrake);

    m_SwivelController.setP(1.0/120.0);
    m_SwivelController.setI(0);
    m_SwivelController.setD(0);
    
}

public boolean getIntakeHasNote() {
   // NOTE: this is intentionally inverted, because the limit switch is normally
   // closed
   return !m_IntakeLimitSwitch.get();
 }

public void setIntakeSpeed(double speed) {

    if(getIntakeHasNote() == true){
      System.out.println("Note is in");
      if(speed > 0){
         m_intakeMotor.set(0);
      }
    } else {
      m_intakeMotor.set(speed);
    }
    writeMetricsToSmartDashboard();
 }
 public void setSwivelSpeed(double speed) {
    m_swivelMotor.set(speed);
    writeMetricsToSmartDashboard();
 }
 public void setSwivelPosition(double rotations){
    m_SwivelController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    writeMetricsToSmartDashboard();
 }

public boolean isIntakeDown(){
   return(m_swivelEncoder.getPosition() >= 29);
}

public boolean isIntakeUp(){
   return(m_swivelEncoder.getPosition() >= 1);
}

 public void writeMetricsToSmartDashboard() {
    SmartDashboard.putNumber("Intake motor set output", m_intakeMotor.get());
    SmartDashboard.putNumber("Swivle set output", m_swivelMotor.get());
    SmartDashboard.putNumber("Swivle motor position", m_swivelEncoder.getPosition());
 }

} 
