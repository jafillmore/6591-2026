// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;



public class ShooterSubsystem extends SubsystemBase {


    private final SparkMax m_shooterSpark;
    private final SparkMax m_shooterSparkfollower;
    
    
    private final RelativeEncoder m_shooterShooterEncoder;
   
    private final SparkClosedLoopController m_shooterShooterClosedLoopController;
    
    private boolean ShooterDebug = true;

    private double shooterSpeedAdjust =0.0;

   


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_shooterSpark = new SparkMax(Constants.ShooterConstants.kshooterShooterCANId, MotorType.kBrushless);
    m_shooterSparkfollower = new SparkMax(Constants.ShooterConstants.kshooterShooterFollowerCANId, MotorType.kBrushless);


    // Retrieve encoders and closed-loop controllers from the shooter sparks as local variables
    m_shooterShooterEncoder = m_shooterSpark.getEncoder();
    

    m_shooterShooterClosedLoopController = m_shooterSpark.getClosedLoopController();
    

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.


 

    m_shooterSpark.configure(Configs.Shooter.shooterConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_shooterSparkfollower.configure(Configs.Shooter.shooterFollowerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


        

  }


   

  @Override
  public void periodic() {
  // This method will be called once per scheduler run
          //if (ShooterDebug) {
          SmartDashboard.putNumber("Speed Adj", shooterSpeedAdjust);
          SmartDashboard.putNumber("Target Speed", m_shooterShooterClosedLoopController.getSetpoint());
          SmartDashboard.putNumber("Actual Speed: ", m_shooterShooterEncoder.getVelocity());
          SmartDashboard.putNumber("Shooter Error: ", (m_shooterShooterClosedLoopController.getSetpoint() - m_shooterShooterEncoder.getVelocity()));
        //}

  }

    //private double shooterSpeedAdjust =0.0;

  public void shooterSpeedUp() {
    shooterSpeedAdjust = shooterSpeedAdjust + ShooterConstants.kshooterSpeedOffset;
    m_shooterShooterClosedLoopController.setSetpoint(ShooterConstants.kshooterShooterSpeed + shooterSpeedAdjust, ControlType.kVelocity);
    
  };

  public void shooterSpeedDown() {
    shooterSpeedAdjust =shooterSpeedAdjust - ShooterConstants.kshooterSpeedOffset;
    m_shooterShooterClosedLoopController.setSetpoint(ShooterConstants.kshooterShooterSpeed + shooterSpeedAdjust, ControlType.kVelocity);
    
  };

  public void setShooterSpeed(double shooterSpeed) {
    m_shooterShooterClosedLoopController.setSetpoint(shooterSpeed+shooterSpeedAdjust, ControlType.kVelocity);
  }

  public void setShooterSpeedFromRange(double range) {
    double shooterSpeed = 0.2929*range*range - 49.207*range + 4813.5; // Quadratic fit for range to speed, from testing data
    m_shooterShooterClosedLoopController.setSetpoint(shooterSpeed, ControlType.kVelocity);
  }


  public void stopShooter() {
    m_shooterSpark.stopMotor();
  }

  public void toggleShooterDebugInfo() {
    ShooterDebug = !ShooterDebug;
  }




}
