// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;

 
// import swervelib.SwerveDrive;
// import swervelib.telemetry.SwerveDriveTelemetry;


/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */






public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
    public double targetYaw;
    public boolean targetVisible;
      // Default hostname is "photonvision", but we changed that to "CAMERA_NAME"
    private PhotonCamera camera;

  public VisionSubsystem() {
    
    
    //Initialize the PhotonCamera
    camera = new PhotonCamera("Pantherpi-Cam1");




   
  }


 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

        // Read in relevant data from the Camera
    targetVisible = false;
    targetYaw = 0.0;
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
          // At least one AprilTag was seen by the camera
          for (var target : result.getTargets()) {
          if (target.getFiducialId() == 10) {
          // Found Tag 7, record its information
            targetYaw = target.getYaw();
            targetVisible = true;
            
           
          }
        }
      }
    }

    SmartDashboard.putBoolean("Target Visible from subsys", targetVisible);
    SmartDashboard.putNumber("Target yaw from subsys",targetYaw);

    
    
  }

  public double getTargetYaw() {
    return targetYaw;
  }



   
}






 
