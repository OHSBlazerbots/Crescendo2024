package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleCameraSubsystem extends SubsystemBase {

    Servo xaxisServo;
    Servo yaxisServo;


    public ExampleCameraSubsystem(){
        xaxisServo = new Servo(1);
        xaxisServo.set(.5);
        xaxisServo.setAngle(75);

        2yaxisServo = new Servo(1);
        yaxisServo.set(.5);
        yaxisServo.setAngle(75);

    
    }


}