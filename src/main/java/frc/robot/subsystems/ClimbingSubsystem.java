// package frc.robot.subsystems;

// import frc.robot.Constants.ClimbingConstants;
// import frc.robot.Constants.ShooterConstants;
// // import frc.robot.subsystems.motor_controllers.PositionPidMotorController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// public class ClimbingSubsystem extends SubsystemBase {
//     // private static final PositionPidMotorController m_climbingController = new PositionPidMotorController(
//     //     ClimbingConstants.kClimbingGains,
//     //     ClimbingConstants.kClimbingMotorPort);
//     // private CANSparkMax m_leadClimbingMotor = new CANSparkMax(ClimbingConstants.kClimbingMotorPort, MotorType.kBrushless);
//     // private CANSparkMax m_secondaryClimbingMotor = new CANSparkMax(ClimbingConstants.kClimbingMotorPort, MotorType.kBrushless);  
//     private CANSparkMax m_rightMotor = new CANSparkMax(ClimbingConstants.kRightClimbingMotorPort, MotorType.kBrushless);
//     private CANSparkMax m_leftMotor = new CANSparkMax(ClimbingConstants.kLeftClimbingMotorPort, MotorType.kBrushless);
//     private CANSparkMax m_climbingMotor;
//     private SparkPIDController m_RightClimbingController = m_rightMotor.getPIDController();
//     private SparkPIDController m_LetfClimbingController = m_leftMotor.getPIDController();
//     private RelativeEncoder m_climbingEncoder; 
//     public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;     
 
//     public ClimbingSubsystem() {
//        m_climbingMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
//        m_climbingController = m_climbingMotor.getPIDController();
//        m_climbingEncoder = m_climbingMotor.getEncoder();
//        m_climbingMotor.restoreFactoryDefaults();

//     //    m_climbingController.zeroSensors();
//     //    m_climbingController.setPositionZero();
//     //    m_leadClimbingMotor.restoreFactoryDefaults();
//     //    m_secondaryClimbingMotor.restoreFactoryDefaults();
//     }
 
//     public void setClimbingPosition(double targetPosition) {
//        double motorPosition = targetPosition;
//        m_leadMotor.setClimbingPosition(motorPosition); //placeholder code
//        m_secondaryMotor.setClimbingPosition(motorPosition); //placeholder code
//        writeMetricsToSmartDashboard();
//     }
 
  
//     public void setClimbingSpeed(double speed) {
//        // TODO: test this logic!!!
//        // stops elavator when it reaches the relative max height to prevent breakage
//        // if (m_climbingController.getCurrentAbsolutePosition() >=
//        // ClimbingConstants.kElevEncoderRotationsAtMaxHeight
//        // && speed < 0) {
//        // return;
//        // }
//        m_leadClimbingMotor.set(speed);
//        m_secondaryClimbingMotor.set(speed);
//        writeMetricsToSmartDashboard();
//     }
 
//     public void writeMetricsToSmartDashboard() {
//        SmartDashboard.putNumber("Climbing Relative Position", m_climbingController.getCurrentRelativePosition());
//        SmartDashboard.putNumber("Climbing Absolute Position", m_climbingController.getCurrentAbsolutePosition());
//        SmartDashboard.putNumber("Climbing Goal", m_climbingController.getTargetPosition());
//        SmartDashboard.putNumber("Climbing Motor set output", m_climbingController.getOutput());
//     }
 
//     public boolean isAtpositionA() {
 
//        return true;
//        // need to finish implementation, hook up hall effect sensors
//     }
 
//     public void zeroSensors() {
//        m_climbingController.zeroSensors();
//     }
 
//     public void setPositionZero() {
//        m_climbingController.setPositionZero();
//     }
 
//     public void setClimbingPosition() {
//        Double targetPosition = SmartDashboard.getNumber("Climbing Goal", 10);
//        setClimbingPosition(targetPosition);
//     }
//  }
