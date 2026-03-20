// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;



public class ShooterSubsystem extends SubsystemBase {


    private final SparkMax m_shooterShooterSpark;
    private final SparkMax m_shooterTurnerSpark;

    private final RelativeEncoder m_shooterShooterEncoder;
    private final RelativeEncoder m_shooterTurnerEncoder;

    private final SparkClosedLoopController m_shooterShooterClosedLoopController;
    private final SparkClosedLoopController m_shooterTurnerClosedLoopController;
    private boolean ShooterDebug = true;

    private double shooterSpeedAdjust =0.0;




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
          SmartDashboard.putNumber("Speed Adj", shooterSpeedAdjust);
          SmartDashboard.putNumber("Target Speed", m_shooterShooterClosedLoopController.getSetpoint());
          SmartDashboard.putNumber("Actual Speed: ", m_shooterShooterEncoder.getVelocity());
          SmartDashboard.putNumber("Shooter Error: ", (m_shooterShooterClosedLoopController.getSetpoint() - m_shooterShooterEncoder.getVelocity()));
        }

  }

    //private double shooterSpeedAdjust =0.0;

  public void shooterSpeedUp() {
    shooterSpeedAdjust =+ ShooterConstants.kshooterSpeedOffset;
    m_shooterShooterClosedLoopController.setSetpoint(ShooterConstants.kshooterShooterSpeed + shooterSpeedAdjust, ControlType.kVelocity);
    
  };

  public void shooterSpeedDown() {
    shooterSpeedAdjust =- ShooterConstants.kshooterSpeedOffset;
    m_shooterShooterClosedLoopController.setSetpoint(ShooterConstants.kshooterShooterSpeed + shooterSpeedAdjust, ControlType.kVelocity);
    
  };

  public void setShooterSpeed(double shooterSpeed) {
    m_shooterShooterClosedLoopController.setSetpoint(shooterSpeed+shooterSpeedAdjust, ControlType.kVelocity);
  }


  public void stopShooter() {
    m_shooterShooterSpark.stopMotor();
  }

  public void toggleShooterDebugInfo() {
    ShooterDebug = !ShooterDebug;
  }




}
