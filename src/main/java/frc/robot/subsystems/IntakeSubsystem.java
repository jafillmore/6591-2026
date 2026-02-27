// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  private final SparkMax m_lowerIntakeSpark;
  private final SparkMax m_upperIntakeSpark; 
    
 

  public IntakeSubsystem() {

    m_lowerIntakeSpark = new SparkMax(IntakeConstants.klowerIntakeCANId, MotorType.kBrushless);
    m_upperIntakeSpark = new SparkMax(IntakeConstants.kupperIntakeCANId, MotorType.kBrushless);


    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_lowerIntakeSpark.configure(Configs.Intake.lowerIntakeConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_upperIntakeSpark.configure(Configs.Intake.upperIntakeConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  

  }


  @Override
  public void periodic() {

  }

  
  public void setIntake(double lowerPower, double upperPower) {
    m_lowerIntakeSpark.set(lowerPower);
    m_upperIntakeSpark.set(upperPower);
  }

  





 



  

}
