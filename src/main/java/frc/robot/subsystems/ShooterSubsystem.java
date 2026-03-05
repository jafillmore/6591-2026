// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;



public class ShooterSubsystem extends SubsystemBase {


    private final SparkMax m_shooterShooterSpark;
    private final SparkMax m_shooterTurnerSpark;

    private final RelativeEncoder m_shooterShooterEncoder;
    private final RelativeEncoder m_shooterTurnerEncoder;

    private final SparkClosedLoopController m_shooterShooterClosedLoopController;
    private final SparkClosedLoopController m_shooterTurnerClosedLoopController;
    private boolean ShooterDebug = true;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_shooterShooterSpark = new SparkMax(Constants.ShooterConstants.kshooterShooterCANId, MotorType.kBrushless);
    m_shooterTurnerSpark = new SparkMax(Constants.ShooterConstants.kshooterTurnerCANId, MotorType.kBrushless);

    // Retrieve encoders and closed-loop controllers from the shooter sparks as local variables
    m_shooterShooterEncoder = m_shooterShooterSpark.getEncoder();
    m_shooterTurnerEncoder = m_shooterTurnerSpark.getEncoder();

    m_shooterShooterClosedLoopController = m_shooterShooterSpark.getClosedLoopController();
    m_shooterTurnerClosedLoopController = m_shooterTurnerSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
 

    m_shooterShooterSpark.configure(Configs.Shooter.shooterShooterConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_shooterTurnerSpark.configure(Configs.Shooter.shooterTurnerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  // This method will be called once per scheduler run

      if (ShooterDebug) {
          SmartDashboard.putNumber("Shooter Speed: ", m_shooterShooterEncoder.getVelocity());
          SmartDashboard.putNumber("Shooter Error: ", (Constants.ShooterConstants.kshooterShooterSpeed - m_shooterShooterEncoder.getVelocity()));
          SmartDashboard.putNumber("Turner Angle: ",  m_shooterTurnerEncoder.getPosition());
      }
    }


  public void setShooterSpeed(double shooterSpeed) {
    m_shooterShooterClosedLoopController.setSetpoint(shooterSpeed, ControlType.kVelocity);
  }

  public void setTurnerAngle(double turnerAngle) {
    m_shooterTurnerClosedLoopController.setSetpoint(turnerAngle, ControlType.kPosition);
  }
  

  public void stopShooter() {
    m_shooterShooterSpark.stopMotor();
  }

  public void toggleShooterDebugInfo() {
    ShooterDebug = !ShooterDebug;
  }

  public void setManualTurretPower(double power) {
    m_shooterTurnerSpark.set(power);
  }

  public void zeroTurretEncoder() {
    m_shooterTurnerEncoder.setPosition(0);
  }






  /**
   * Aim the turret at a fixed field location.
   *
   * @param fieldTarget The target pose on the field to aim at (x,y used; rotation ignored)
   * @param drive The DriveSubsystem to get the robot's current pose from
   */
  public void aimAtFieldLocation(Pose2d fieldTarget, DriveSubsystem drive) {
    Pose2d robotPose = drive.getPose();
    // Compute vector from robot to target in field coordinates
    double dx = fieldTarget.getX() - robotPose.getX();
    double dy = fieldTarget.getY() - robotPose.getY();
    // Desired heading in field frame
    double desiredHeading = Math.atan2(dy, dx);
    // Robot heading in field frame
    double robotHeading = robotPose.getRotation().getRadians();
    // Turret angle relative to robot forward (radians)
    double turretAngleRad = desiredHeading - robotHeading;
    // Normalize to [-pi, pi]
    turretAngleRad = Math.atan2(Math.sin(turretAngleRad), Math.cos(turretAngleRad));

    // Convert radians to motor rotations (assuming encoder reports rotations)
    double turretDegrees = turretAngleRad * 180.0 / Math.PI;

    // Command the turret controller to the desired position
    setTurnerAngle(turretDegrees);

    // Publish debug info
    if (ShooterDebug) {
      SmartDashboard.putNumber("AutoAim Target X", fieldTarget.getX());
      SmartDashboard.putNumber("AutoAim Target Y", fieldTarget.getY());
      SmartDashboard.putNumber("AutoAim DesiredHeadingdeg", desiredHeading * 180 / Math.PI);
      SmartDashboard.putNumber("AutoAim TurretTargetAngle", turretAngleRad);
      SmartDashboard.putNumber("AutoAim TurretActualAngle", m_shooterTurnerEncoder.getPosition());
    }
  }

}
