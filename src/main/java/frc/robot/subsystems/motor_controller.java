// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class motor_controller {
//     double kP = 0.15; 
//     double kI = 0.0;
//     double kD = 0.0; 
//     double kIz = 0.0; 
//     double kFF = 1.0; 
//     double kMaxOutput = 1.0; 
//     double kMinOutput = -1.0;
//     double maxRPM = 5700.0;

//     public motor_controller(){
//     m_controller.setP(kP);
//     m_controller.setI(kI);
//     m_controller.setD(kD);
//     m_controller.setIZone(kIz);
//     m_controller.setFF(kFF);
//     m_controller.setOutputRange(kMinOutput, kMaxOutput);

//     double p = SmartDashboard.getNumber("P Gain", 0);
//     double i = SmartDashboard.getNumber("I Gain", 0);
//     double d = SmartDashboard.getNumber("D Gain", 0);
//     double iz = SmartDashboard.getNumber("I Zone", 0);
//     double ff = SmartDashboard.getNumber("Feed Forward", 0);
//     double max = SmartDashboard.getNumber("Max Output", 0);
//     double min = SmartDashboard.getNumber("Min Output", 0);

//     if((p != kP)) { m_controller.setP(p); kP = p; }
//     if((i != kI)) { m_controller.setI(i); kI = i; }
//     if((d != kD)) { m_controller.setD(d); kD = d; }
//     if((iz != kIz)) { m_controller.setIZone(iz); kIz = iz; }
//     if((ff != kFF)) { m_controller.setFF(ff); kFF = ff; }
//     if((max != kMaxOutput) || (min != kMinOutput)) { 
//       m_controller.setOutputRange(min, max); 
//       kMinOutput = min; kMaxOutput = max; 
//     }
// }
// }
