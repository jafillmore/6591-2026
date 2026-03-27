// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase {
 
  /** Creates a new Climber. */
  private final SparkMax m_blackClimberSpark; 
  private final SparkMax m_orangeClimberSpark;  
  private final SparkMax m_blackArmSpark;
  private final SparkMax m_orangeArmSpark;

  private final RelativeEncoder m_blackClimberEncoder;
  private final RelativeEncoder m_orangeClimberEncoder;
  private final RelativeEncoder m_blackArmEncoder;
  private final RelativeEncoder m_orangeArmEncoder;

  private final SparkClosedLoopController m_blackClimberClosedLoopController;
  private final SparkClosedLoopController m_orangeClimberClosedLoopController;
  private final SparkClosedLoopController m_blackArmClosedLoopController;
  private final SparkClosedLoopController m_orangeArmClosedLoopController;
    //Variables for System Debugging
  private boolean ClimberSystemDebug = true;

  public ClimberSubsystem() {

    m_blackClimberSpark = new SparkMax(ClimberConstants.kblackClimberCANId, MotorType.kBrushless);
    m_orangeClimberSpark = new SparkMax(ClimberConstants.korangeClimberCANId, MotorType.kBrushless);
    m_blackArmSpark = new SparkMax(ClimberConstants.kblackArmCANId, MotorType.kBrushless);
    m_orangeArmSpark = new SparkMax(ClimberConstants.korangeArmCANId, MotorType.kBrushless);

    m_blackClimberEncoder = m_blackClimberSpark.getEncoder();
    m_orangeClimberEncoder = m_orangeClimberSpark.getEncoder();
    m_blackArmEncoder = m_blackArmSpark.getEncoder();
    m_orangeArmEncoder = m_orangeArmSpark.getEncoder();


    m_blackClimberClosedLoopController = m_blackClimberSpark.getClosedLoopController();
    m_orangeClimberClosedLoopController = m_orangeClimberSpark.getClosedLoopController();
    m_blackArmClosedLoopController = m_blackArmSpark.getClosedLoopController();
    m_orangeArmClosedLoopController = m_orangeArmSpark.getClosedLoopController();

    m_blackClimberSpark.configure(Configs.Climber.blackclimberConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    m_orangeClimberSpark.configure(Configs.Climber.orangeclimberConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    m_blackArmSpark.configure(Configs.Climber.blackarmConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    m_orangeArmSpark.configure(Configs.Climber.orangearmConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    climberDebugInfo();
  
  }

  


  public void setBlackClimber (Double blackclimberPosition) {
    m_blackClimberClosedLoopController.setSetpoint(blackclimberPosition, ControlType.kPosition);
  }
  //
  public void setBlackArm(Double blackarmPower) {
    m_blackArmSpark.set(blackarmPower);
  }
  
  public void setOrangeClimber (Double orangeclimberPosition) {
    m_orangeClimberClosedLoopController.setSetpoint(orangeclimberPosition, ControlType.kPosition);
  }
 //
  public void setOrangeArm(Double orangearmPower) {
    m_orangeArmSpark.set(orangearmPower);
  }
 
  public void stopClimber() {
    m_blackClimberSpark.stopMotor();
    m_orangeClimberSpark.stopMotor();
    m_blackArmSpark.stopMotor();
    m_orangeArmSpark.stopMotor();
  }

  public void resetClimberEncoders() {
    m_blackClimberEncoder.setPosition(0);
    m_orangeClimberEncoder.setPosition(0);
  }

  public void zeroClimber() {
    m_blackClimberSpark.set(0.04);
    m_orangeClimberSpark.set(0.04);
    Timer.delay(1);
    resetClimberEncoders();
    stopClimber();
  }

  public void climberDown() {
    m_blackArmSpark.stopMotor();
    m_orangeArmSpark.stopMotor();
    m_blackClimberSpark.set(-0.05);
    m_orangeClimberSpark.set(-0.05);
    Timer.delay(1);
    stopClimber();
  }



  public void toggleClimberDebugInfo() {
    ClimberSystemDebug = !ClimberSystemDebug;
    return; 
  }

  //  System Debug Info to display
  public void climberDebugInfo(){
    if (ClimberSystemDebug) {
      SmartDashboard.putNumber(  "LC Actual", m_blackClimberEncoder.getPosition());
      SmartDashboard.putNumber(  "RC Actual", m_orangeClimberEncoder.getPosition());
    }
  }

}
