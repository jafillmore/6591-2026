// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;


 
// import swervelib.SwerveDrive;
// import swervelib.telemetry.SwerveDriveTelemetry;


/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */






public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  
  public boolean targetVisible;
  private boolean VisionSystemDebug = true;
  private boolean AimingDebug = true;
  public double targetYaw;
  private double turnError = 0.0;
  public double range = 0.0;
  public double twistPower = 0.0;

  // Default hostname is "photonvision", but we changed that to "CAMERA_NAME"
  //private PhotonCamera camera;
  private final DriveSubsystem m_driveSubsystem;
  public final AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonCamera poseCamera1;
  private final PhotonCamera poseCamera2;
  //private final PhotonCamera targetingCamera;

  //public Optional<PhotonPipelineResult> targetingCamera1Result;

  private final PhotonPoseEstimator poseCamera1PoseEstimator;
  private final PhotonPoseEstimator poseCamera2PoseEstimator;
  
  // Flag to ensure we only reset odometry once at startup from vision
  private boolean initialPoseSet = false;
  
  // Latest vision pose and timestamp
  private java.util.Optional<edu.wpi.first.math.geometry.Pose2d> latestVisionPose = java.util.Optional.empty();
  private double latestVisionTimestamp = 0.0;
  
  // PID controller for aiming at the center of the hub
  public final PIDController aimingController = new PIDController(VisionConstants.kPAimingController, 0, VisionConstants.kDAimingController);
  
  
  // setup cameras
  private PhotonCameraSim poseCamera1Sim;
  private PhotonCameraSim poseCamera2Sim;


  public VisionSubsystem(DriveSubsystem d_subsystem) {

    m_driveSubsystem = d_subsystem;
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    poseCamera1 = new PhotonCamera(Constants.PoseCamera1.name);
    poseCamera2 = new PhotonCamera(Constants.PoseCamera2.name);
    //targetingCamera = new PhotonCamera(Constants.PoseCamera2.name);

    poseCamera1PoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.PoseCamera1.location);
    poseCamera2PoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.PoseCamera2.location);
    poseCamera1PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseCamera2PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


  }

    /**
    * Gets the last procesesd frame captured by camera
    *
    * @param camera Desired camera to get result from
    * @return Targets in the frame.
    */
    private Optional<PhotonPipelineResult> getPipelineResults(PhotonCamera camera) {
      var results = camera.getAllUnreadResults();
      if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        SmartDashboard.putNumber("Front Camera Latency", result.getTimestampSeconds());
        if (result.hasTargets()) {
          // select last result with targets
          return Optional.of(result);
        }
      }
      return Optional.empty();
    }

    /**
     * Update estaimated robot pose based on given pipeline result.
     *
     * @param camera Pose Camera
     * @param poseEstimator Pose estimator
     */
    private void updateGlobalPose(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
      for (var result : camera.getAllUnreadResults()) {
        Optional<EstimatedRobotPose> curPose = poseEstimator.update(result);
        if (curPose.isPresent()) {
          var pose2d = curPose.get().estimatedPose.toPose2d();
          double timestamp = curPose.get().timestampSeconds;
          // Store latest vision pose for dashboard and possible manual reset
          latestVisionPose = java.util.Optional.of(pose2d);
          latestVisionTimestamp = timestamp;

          // Feed the vision measurement into the drive pose estimator
          m_driveSubsystem.updateVisionPose(pose2d, timestamp);

          // If we haven't set the robot's initial pose yet, and we are disabled, use this vision pose to reset odometry
          if (!initialPoseSet && DriverStation.isDisabled()) {
            m_driveSubsystem.resetOdometry(pose2d);
            initialPoseSet = true;
            SmartDashboard.putBoolean("InitialPoseSetFromVision", true);
            System.out.println("[VisionSubsystem] Initial odometry reset from vision pose at timestamp: " + timestamp);
          }
        }
      }
    }

    //Toggle Vision Debug Info
    public void toggleVisionDebugInfo (){
      VisionSystemDebug = !VisionSystemDebug;
      return; 
    }

  
   //Toggle Target Debug Info
    public void toggleTargetDebugInfo (){
      AimingDebug = !AimingDebug;
      return; 
    }


  /**
   * Aim at a fixed field location.
   *
   * @param fieldTarget The target pose on the field to aim at (x,y used; rotation ignored)
   * @param drive The DriveSubsystem to get the robot's current pose from
   */
  public double[] aimAtFieldLocation(DriveSubsystem drive) {
    Pose2d robotPose = drive.getPose();

    double ax;
    double ay;
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      ax = ShooterConstants.kRedHubXPosition;
      ay = ShooterConstants.kRedHubYPosition;
    } else {
      ax = ShooterConstants.kBlueHubXPosition;
      ay = ShooterConstants.kBlueHubYPosition;        
    }

    // Define the target pose on the field (x and y in meters, rotation ignored for aiming)
    Pose2d fieldTarget = new Pose2d(ax, ay, new Rotation2d(0));

    Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, fieldTarget);
    double turnError = MathUtil.angleModulus(targetYaw.getRadians()); // Since we want to aim the front of the robot, the error is just the yaw to the target pose
  
    double range = PhotonUtils.getDistanceToPose(robotPose, fieldTarget);


    double turnPower = aimingController.calculate(turnError*180/Math.PI, 0);
    double powerPlusRange[] = {turnPower, range};
    
    // Normalize to [-pi, pi]
    // turnError = Math.atan2(Math.sin(turnError), Math.cos(turnError));


    // Publish debug info
    if (AimingDebug) {
      SmartDashboard.putNumber("Target X", ax);
      SmartDashboard.putNumber("Target Y", ay);
      SmartDashboard.putNumber("Target Yaw", targetYaw.getDegrees());
      SmartDashboard.putNumber("RobotAngleError", turnError * 180 / Math.PI);
    }

    return powerPlusRange;

  }


  public double aimAtTarget(DriveSubsystem drive) {

// Read in relevant data from the Camera
    targetVisible = false;
    targetYaw = 0.0;
    int targetID = /*(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) ? VisionConstants.kRedTargetID :*/ VisionConstants.kBlueTargetID;
    var results = poseCamera2.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
          // At least one AprilTag was seen by the camera
          for (var target : result.getTargets()) {
          if (target.getFiducialId() == targetID) {
          // Found Tag 7, record its information
            targetYaw = target.getYaw();
            targetVisible = true;
            //double range = target.getRange();

            twistPower=targetYaw*VisionConstants.kAimAtTarget_kP;

           
          }
        }
      }
    }

    return twistPower;

  }





    @Override
  public void periodic() {
    // This method will be called once per scheduler run
     // update the pipeline result for targeting cameras
    //targetingCamera1Result = getPipelineResults(targetingCamera1);
    // update the pose estimators
    updateGlobalPose(poseCamera1, poseCamera1PoseEstimator);
    updateGlobalPose(poseCamera2, poseCamera2PoseEstimator);

// Read in relevant data from the Camera
    targetVisible = false;
    targetYaw = 0.0;
    int targetID = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) ? VisionConstants.kRedTargetID : VisionConstants.kBlueTargetID;
    var results = poseCamera1.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
          // At least one AprilTag was seen by the camera
          for (var target : result.getTargets()) {
          if (target.getFiducialId() == targetID) {
          // Found Tag 7, record its information
            targetYaw = target.getYaw();
            targetVisible = true;
            //double range = target.getRange();

            twistPower=targetYaw*VisionConstants.kAimAtTarget_kP;
           
          }
        }
      }
    }

   

    // Publish diagnostic info and latest vision pose for Shuffleboard
    if (VisionSystemDebug) {
      SmartDashboard.putBoolean("poseCamera1Connected", poseCamera1.isConnected());
      SmartDashboard.putBoolean("poseCamera2Connected", poseCamera2.isConnected());
      //SmartDashboard.putBoolean("TargetingCamera1Connnected", targetingCamera1.isConnected());
    }

    // Publish latest vision pose if available
    if (latestVisionPose.isPresent()) {
      var p = latestVisionPose.get();
      SmartDashboard.putNumber("Vision Pose X", p.getX());
      SmartDashboard.putNumber("Vision Pose Y", p.getY());
      SmartDashboard.putNumber("Vision Pose RotDeg", p.getRotation().getDegrees());
      SmartDashboard.putNumber("Vision Pose Timestamp", latestVisionTimestamp);
      SmartDashboard.putNumber("Twist Power", twistPower);
      SmartDashboard.putNumber("Target Yaw", targetYaw);
      SmartDashboard.putBoolean("Target Found", targetVisible);
      SmartDashboard.putBoolean("Vision Pose Available", true);
    } else {
      SmartDashboard.putBoolean("Vision Pose Available", false);
    }

    // Provide a Shuffleboard/SmartDashboard button named "Force Reset Odometry". If set true,
    // perform a one-time reset to the latest vision pose (if available) and clear the button.
    boolean forceReset = SmartDashboard.getBoolean("Force Reset Odometry", false);
    if (forceReset) {
      if (latestVisionPose.isPresent()) {
        m_driveSubsystem.resetOdometry(latestVisionPose.get());
        initialPoseSet = true;
        SmartDashboard.putBoolean("InitialPoseSetFromVision", true);
        System.out.println("[VisionSubsystem] Force-reset odometry from vision pose via Shuffleboard at timestamp: " + latestVisionTimestamp);
      } else {
        System.out.println("[VisionSubsystem] Force reset requested but no vision pose available yet.");
      }
      // Clear the dashboard button so repeated resets require another manual press
      SmartDashboard.putBoolean("Force Reset Odometry", false);
    } 
    
    






  }

 

}






 
