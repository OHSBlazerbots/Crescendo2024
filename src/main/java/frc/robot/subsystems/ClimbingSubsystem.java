package frc.robot.subsystems;

import frc.robot.Constants.ClimbingConstants;
import frc.robot.subsystems.motorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimbingSubsystem extends SubsystemBase {

    // private static final SparkPIDController m_shooterController = new SparkPIDController(
    //      ShooterConstants.kShooterGains,
    //      ShooterConstants.kShooterMotorPort);
    private CANSparkMax m_rightMotor = new CANSparkMax(ClimbingConstants.kRightClimbingMotorPort, MotorType.kBrushless);
    private CANSparkMax m_leftMotor = new CANSparkMax(ClimbingConstants.kLeftClimbingMotorPort, MotorType.kBrushless);
    private SparkPIDController m_climbingRightController = m_rightMotor.getPIDController();
   private SparkPIDController m_climbingLeftController = m_leftMotor.getPIDController();


    public motorController rightClimber = new motorController(m_climbingRightController, 0.15, 0, 0, 0, 1, 1, -1, 5700);
    public motorController leftClimber = new motorController(m_climbingLeftController, 0.15, 0, 0, 0, 1, 1, -1, 5700);

    

public ClimbingSubsystem() {
    



    // m_shooterController.zeroSensors();
    // m_shooterController.setPositionZero();
    m_rightMotor.restoreFactoryDefaults();
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_leftMotor.setIdleMode(IdleMode.kBrake);

     // Current limiting
     /*int TIMEOUT_MS = 10;
     m_Controller.getMotor().configPeakCurrentLimit(20, TIMEOUT_MS); // 20 Amps
     m_shooterController.getMotor().configPeakCurrentDuration(200, TIMEOUT_MS); // 200ms
     m_shooterController.getMotor().configContinuousCurrentLimit(20, TIMEOUT_MS); // 20 Amps
     */
}

public void setClimberSpeed(double speed) {
    // speed = SmartDashboard.getNumber("Shooter/Speed Output", 0);
   //  System.out.println("speed=" + speed);
   //  System.out.println("dashboard=" + SmartDashboard.getNumber("Shooter/Speed Output", 0));
    m_rightMotor.set(speed);
    m_leftMotor.set(speed);
    writeMetricsToSmartDashboard();
 }

public void setLeftClimberSpeed(double leftSpeed){
   m_leftMotor.set(leftSpeed);
}

public void setRightClimberSpeed(double rightSpeed){
   m_leftMotor.set(rightSpeed);
}

 public void setClimberPosition(double rotations){
    m_climbingRightController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    m_climbingLeftController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    writeMetricsToSmartDashboard();
 }


 public void writeMetricsToSmartDashboard() {
    rightClimber.writeMetricsToSmartDashboard();
    leftClimber.writeMetricsToSmartDashboard();
    SmartDashboard.putNumber("Motor set output", m_rightMotor.get());
    SmartDashboard.putNumber("Motor set output", m_leftMotor.get());
 }

}

