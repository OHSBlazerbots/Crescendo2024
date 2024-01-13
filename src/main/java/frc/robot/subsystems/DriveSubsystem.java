package frc.robot.subsystems;
import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;


public class DriveSubsystem {
double maximumSpeed = Units.feetToMeters(4.5);
File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
public DriveSubsystem()throws IOException{
    SwerveDrive  swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
}
}
