package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.networktables.NetworkTableEntry;

public class CameraSubsystem extends SubsystemBase {
    UsbCamera frontBumper;
    UsbCamera ShooterCam;
    NetworkTableEntry cameraSelection;
    int currentCameraIndex = 0;
    VideoSink cameraServer;
    UsbCamera[] allCameras;

    public CameraSubsystem() {
        frontBumper = CameraServer.startAutomaticCapture(0); // 0 is placeholder
        ShooterCam = CameraServer.startAutomaticCapture(1); // 1 is placeholder
        cameraServer = CameraServer.getServer();
        allCameras = new UsbCamera[] { frontBumper,ShooterCam };
    }

    public void nextCameraSelection() {
        currentCameraIndex += 1;
        currentCameraIndex = currentCameraIndex % allCameras.length;
        System.out.println("current camera index:" + currentCameraIndex);

        UsbCamera currentCamera = allCameras[currentCameraIndex];
        cameraServer.setSource(currentCamera);
    }
}
