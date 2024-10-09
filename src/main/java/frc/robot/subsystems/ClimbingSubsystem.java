package frc.robot.subsystems;

import frc.robot.Constants.ClimbingConstants;
import frc.robot.subsystems.motorController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimbingSubsystem extends SubsystemBase {

    private CANSparkMax m_rightMotor = new CANSparkMax(ClimbingConstants.kRightClimbingMotorPort, MotorType.kBrushless);
    private CANSparkMax m_leftMotor = new CANSparkMax(ClimbingConstants.kLeftClimbingMotorPort, MotorType.kBrushless);
    private SparkPIDController m_climbingRightController = m_rightMotor.getPIDController();
    private SparkPIDController m_climbingLeftController = m_leftMotor.getPIDController();
    private RelativeEncoder m_leftClimbEncoder = m_leftMotor.getEncoder();
    private RelativeEncoder m_rightClimbEncoder = m_rightMotor.getEncoder();


public ClimbingSubsystem() {

float rightForwardLimit = (float) SmartDashboard.getNumber("Right forward Soft Limit", 700);
float rightReverseLimit = (float)SmartDashboard.getNumber("Right reverse Soft Limit", -700);
float leftForwardLimit = (float) SmartDashboard.getNumber("Left forward Soft Limit", 700);
float leftReverseLimit = (float)SmartDashboard.getNumber("Left reverse Soft Limit", -700);

SmartDashboard.putNumber("Right forward Soft Limit", rightForwardLimit);
SmartDashboard.putNumber("Right reverse Soft Limit", rightReverseLimit);
SmartDashboard.putNumber("Left forward Soft Limit", leftForwardLimit);
SmartDashboard.putNumber("Left reverse Soft Limit", leftReverseLimit);
    
    m_rightMotor.restoreFactoryDefaults();
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
   m_rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false); 
   m_rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, rightForwardLimit);
   m_rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, rightReverseLimit);
   m_leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
   m_leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
   m_leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, leftForwardLimit);
   m_leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, leftReverseLimit);
    // Current limiting
     /*int TIMEOUT_MS = 10;
     m_Controller.getMotor().configPeakCurrentLimit(20, TIMEOUT_MS); // 20 Amps
     m_shooterController.getMotor().configPeakCurrentDuration(200, TIMEOUT_MS); // 200ms
     m_shooterController.getMotor().configContinuousCurrentLimit(20, TIMEOUT_MS); // 20 Amps
     */
}

public void setClimberSpeed(double speed) {
    m_rightMotor.set(speed);
    m_leftMotor.set(speed);
 }

public void setLeftClimberSpeed(double leftSpeed){
   m_leftMotor.set(leftSpeed);
}

public void setRightClimberSpeed(double rightSpeed){
   m_rightMotor.set(rightSpeed);
}

 public void setClimberPosition(double rotations){
    m_climbingRightController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    m_climbingLeftController.setReference(rotations, CANSparkMax.ControlType.kPosition);
 }
 @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber right Motor set output", m_rightMotor.get());
    SmartDashboard.putNumber("Climber left Motor set output", m_leftMotor.get());
    SmartDashboard.putNumber("Climber left motor position", m_leftClimbEncoder.getPosition());
    SmartDashboard.putNumber("Climber right motor position", m_rightClimbEncoder.getPosition());
    
  }




}

