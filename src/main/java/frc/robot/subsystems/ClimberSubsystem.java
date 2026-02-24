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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase {
 
  /** Creates a new Climber. */
  private final SparkMax m_leftClimberSpark; 
  private final SparkMax m_rightClimberSpark;  
 

  private final RelativeEncoder m_leftClimberEncoder;
  private final RelativeEncoder m_rightClimberEncoder;


  private final SparkClosedLoopController m_leftClimberClosedLoopController;
  private final SparkClosedLoopController m_rightClimberClosedLoopController;
  
    //Variables for System Debugging
  private boolean ClimberSystemDebug = true;

  public ClimberSubsystem() {

    m_leftClimberSpark = new SparkMax(ClimberConstants.kleftWristCANId, MotorType.kBrushless);
    m_rightClimberSpark = new SparkMax(ClimberConstants.krightWristCANId, MotorType.kBrushless);
    
    
    m_leftClimberEncoder = m_leftClimberSpark.getEncoder();
    m_rightClimberEncoder = m_rightClimberSpark.getEncoder();

    m_leftClimberClosedLoopController = m_leftClimberSpark.getClosedLoopController();
    m_rightClimberClosedLoopController = m_rightClimberSpark.getClosedLoopController();
    
    m_leftClimberSpark.configure(Configs.Climber.leftwristConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    m_rightClimberSpark.configure(Configs.Climber.rightwristConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    toggleClimberDebugInfo();
  
  }

  


  public void setLeftWrist (int leftwristPosition) {
    m_leftClimberClosedLoopController.setReference(leftwristPosition, ControlType.kPosition);
  }
  
  public void setRightWrist (int rightwristPosition) {
    m_rightClimberClosedLoopController.setReference(rightwristPosition, ControlType.kPosition);
  }
 


  //  System Debug Info to display
  public void toggleClimberDebugInfo(){
    if (ClimberSystemDebug) {
      // IMU Status
      SmartDashboard.putNumber(  "Left Climber Actual Position", m_leftClimberEncoder.getPosition());
      SmartDashboard.putNumber(  "Right Climber Actual Postion", m_rightClimberEncoder.getPosition());
  


   
    }
  }

}
