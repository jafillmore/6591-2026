// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;


 
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
  private boolean VisionSystemDebug = false;
  // Default hostname is "photonvision", but we changed that to "CAMERA_NAME"
  //private PhotonCamera camera;
  private final DriveSubsystem m_driveSubsystem;
  public final AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonCamera poseCamera1;
  private final PhotonCamera poseCamera2;
  //private final PhotonCamera targetingCamera1;

  public Optional<PhotonPipelineResult> targetingCamera1Result;

  private final PhotonPoseEstimator poseCamera1PoseEstimator;
  private final PhotonPoseEstimator poseCamera2PoseEstimator;
  // Flag to ensure we only reset odometry once at startup from vision
  private boolean initialPoseSet = false;
  // Latest vision pose and timestamp
  private java.util.Optional<edu.wpi.first.math.geometry.Pose2d> latestVisionPose = java.util.Optional.empty();
  private double latestVisionTimestamp = 0.0;
  private double turnError = 0.0;
  private boolean AimingDebug = false;
  public double range = 0.0;

    // Simulation Config
  // A vision system sim labelled as "pose and targeting" in NetworkTables
  private VisionSystemSim poseVisionSim;
  private VisionSystemSim targetingVisionSim;

  /*
  // A 0.5 x 0.25 meter rectangular target
  private final TargetModel targetModel = new TargetModel(0.5, 0.25);
  // The pose of where the target is on the field.
  // Its rotation determines where "forward" or the target x-axis points.
  // Let's say this target is flat against the far wall center, facing the blue driver stations.
  private final Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
  // The given target model at the given pose
  private final VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
  */
  
  // setup cameras
  private final SimCameraProperties PoseCameraProp = new SimCameraProperties();
 
  private PhotonCameraSim poseCamera1Sim;
  private PhotonCameraSim poseCamera2Sim;






  public VisionSubsystem(DriveSubsystem d_subsystem) {

    m_driveSubsystem = d_subsystem;
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    poseCamera1 = new PhotonCamera(Constants.PoseCamera1.name);
    poseCamera2 = new PhotonCamera(Constants.PoseCamera2.name);
    //targetingCamera1 = new PhotonCamera(Constants.TargetingCamera1.name);

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

    if (Robot.isSimulation()) {
      simulationInit();
    }
  }

  private void simulationInit() {
    // setup simulation for vision system
    poseVisionSim = new VisionSystemSim("pose");
    poseVisionSim.addAprilTags(aprilTagFieldLayout);



    // Set the properties of the camera
 
    PoseCameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(70));

    // Approximate detection noise with average and standard deviation error in pixels.

    PoseCameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).

    PoseCameraProp.setFPS(50);

    // The average and standard deviation in milliseconds of image data latency.

    PoseCameraProp.setAvgLatencyMs(35);

    PoseCameraProp.setLatencyStdDevMs(5);

    // initialize the cameras
    poseCamera1Sim = new PhotonCameraSim(poseCamera1, PoseCameraProp);
    poseCamera2Sim = new PhotonCameraSim(poseCamera2, PoseCameraProp);


    // Set Camera locations and add them to the vision simulation
    poseVisionSim.addCamera(poseCamera1Sim, Constants.PoseCamera1.location);
    poseVisionSim.addCamera(poseCamera2Sim, Constants.PoseCamera2.location);

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
   * Aim the turret at a fixed field location.
   *
   * @param fieldTarget The target pose on the field to aim at (x,y used; rotation ignored)
   * @param drive The DriveSubsystem to get the robot's current pose from
   */
  public double aimAtFieldLocation(DriveSubsystem drive) {
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
    // Compute vector from robot to target in field coordinates
    double dx = ax - robotPose.getX();
    double dy = ay - robotPose.getY();
    // Desired heading in field frame
    double desiredHeading = Math.atan2(dy, dx);
    // Robot heading in field frame
    double robotHeading = robotPose.getRotation().getRadians();
    // Robot angle relative to target (radians)
    turnError = desiredHeading - robotHeading;

    
    // Normalize to [-pi, pi]
    // turnError = Math.atan2(Math.sin(turnError), Math.cos(turnError));


    // Publish debug info
    if (AimingDebug) {
      SmartDashboard.putNumber("Target X", ax);
      SmartDashboard.putNumber("Target Y", ay);
      SmartDashboard.putNumber("DesiredHeadingdeg", desiredHeading * 180 / Math.PI);
      SmartDashboard.putNumber("RobotAngleError", turnError);
    }

    return turnError;

  }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
     // update the pipeline result for targeting cameras
    //targetingCamera1Result = getPipelineResults(targetingCamera1);
    // update the pose estimators
    updateGlobalPose(poseCamera1, poseCamera1PoseEstimator);
    updateGlobalPose(poseCamera2, poseCamera2PoseEstimator);

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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Update with the simulated drivetrain pose. This should be called every loop in simulation.
    poseVisionSim.update(m_driveSubsystem.getPose());
    targetingVisionSim.update(m_driveSubsystem.getPose());
  }  

}






 
