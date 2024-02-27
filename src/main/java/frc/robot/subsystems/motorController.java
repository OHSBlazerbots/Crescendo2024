package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class motorController {

  double kP; 
  double kI;
  double kD; 
  double kIz; 
  double kFF; 
  double kMaxOutput; 
  double kMinOutput;
  double maxRPM;

    public motorController(SparkPIDController m_controller, double kP, double kI, double kD, double kIz, double kFF, double kMaxOutput, double kMinOutput,    double maxRPM){
    
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kIz = kIz;
      this.kFF = kFF;
      this.kMaxOutput = kMaxOutput;
      this.kMinOutput = kMinOutput;
      this.maxRPM = maxRPM;

    m_controller.setP(kP);
    m_controller.setI(kI);
    m_controller.setD(kD);
    m_controller.setIZone(kIz);
    m_controller.setFF(kFF);
    m_controller.setOutputRange(kMinOutput, kMaxOutput);

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    if((p != kP)) { m_controller.setP(p); kP = p; }
    if((i != kI)) { m_controller.setI(i); kI = i; }
    if((d != kD)) { m_controller.setD(d); kD = d; }
    if((iz != kIz)) { m_controller.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_controller.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_controller.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
}

public void writeMetricsToSmartDashboard() {

  SmartDashboard.putNumber("P Gain", kP);
  SmartDashboard.putNumber("I Gain", kI);
  SmartDashboard.putNumber("D Gain", kD);
  SmartDashboard.putNumber("I Zone", kIz);
  SmartDashboard.putNumber("Feed Forward", kFF);
  SmartDashboard.putNumber("Max Output", kMaxOutput);
  SmartDashboard.putNumber("Min Output", kMinOutput);
}
}
