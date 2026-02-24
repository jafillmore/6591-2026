// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  private final SparkMax m_troughSpark;
  private final SparkMax m_elevatorSpark; 
    
  private final AbsoluteEncoder m_troughEncoder;
  private final RelativeEncoder m_elevatorEncoder;

  private final SparkClosedLoopController m_troughClosedLoopController;
  private final SparkClosedLoopController m_elevatorClosedLoopController;
  private final ServoHub m_servoHub; 
  private final ServoChannel m_Channel0;

  //Variables for System Debugging
  private boolean CoralSystemDebug = true;

  public Command setTroughForClearingAlgeaCommand() {
    return this.startEnd(
      () -> this.setTrough(CoralConstants.ktAlgeaAngle), // Start
      () -> this.setTrough(CoralConstants.ktAlgeaAngle));  // End
  }
  
  public Command clearAlgeaCommand() {
    return this.startEnd(
      () -> this.setElevator(CoralConstants.kElevatorL4),  // Start
      () -> this.setTrough(CoralConstants.kElevatorL4));  //End
  }

  public Command dumpCoralCommand() {
    return this.startEnd(
      () -> this.setTrough(CoralConstants.ktL4Angle), // Start
      () -> this.setTrough(CoralConstants.ktL4Angle));
  }


 public Command stowCoralCommand() {
    return this.startEnd(
      () -> setBothELTR(CoralConstants.kElevatorStow, CoralConstants.ktStowAngle),
      () -> setBothELTR(CoralConstants.kElevatorStow, CoralConstants.ktStowAngle));
 }

 public Command loadCommand() {
    return this.startEnd(
      () -> setTrough(CoralConstants.ktLoadAngle),
      () -> setTrough(CoralConstants.ktLoadAngle));
 }


  



  public CoralSubsystem() {

    m_troughSpark = new SparkMax(CoralConstants.ktroughCANId, MotorType.kBrushless);
    m_elevatorSpark = new SparkMax(CoralConstants.kelevatorCANId, MotorType.kBrushless);

    m_troughEncoder = m_troughSpark.getAbsoluteEncoder();
    m_elevatorEncoder = m_elevatorSpark.getEncoder();

    m_troughClosedLoopController = m_troughSpark.getClosedLoopController();
    m_elevatorClosedLoopController = m_elevatorSpark.getClosedLoopController();

    m_servoHub = new ServoHub (CoralConstants.kServohubCANId);
    m_Channel0 = m_servoHub.getServoChannel(ChannelId.kChannelId0);

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_troughSpark.configure(Configs.Coral.troughConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_elevatorSpark.configure(Configs.Coral.elevatorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  
    m_servoHub.configure(Configs.Coral.servoConfig,ServoHub.ResetMode.kNoResetSafeParameters);
    m_servoHub.setBankPulsePeriod (ServoHub.Bank.kBank0_2,1500);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    coaralDebugInfo();
  }

  

  public void setElevator (int elevatorPosition) {
    m_elevatorClosedLoopController.setReference(elevatorPosition, ControlType.kPosition);
  }

  



  public void setTrough (double troughPosition) {
    m_troughClosedLoopController.setReference(troughPosition,ControlType.kPosition);
  }



  public void setBothELTR (int elevatorPsn, double troughPsn) {
    setElevator(elevatorPsn);
    setTrough(troughPsn);
  }



  public void zeroElevator () {
    m_elevatorSpark.set (0.02);
    Timer.delay (0.7);
    m_elevatorEncoder.setPosition(0);
    m_elevatorSpark.set(0);
  }

  public void pinSet (int pinpsn) {
    m_Channel0.setPulseWidth(pinpsn);
  }


  //Toggle Drive Debug Info
  public void toggleCoralebugInfo (){
    CoralSystemDebug = !CoralSystemDebug;
    return;    

  }

//  System Debug Info to display
  public void coaralDebugInfo(){
    if (CoralSystemDebug) {
      // IMU Status
      SmartDashboard.putNumber(  "Trough Actual Position", m_troughEncoder.getPosition());
      ///SmartDashboard.putNumber("Target Postion",
      SmartDashboard.putNumber(  "Elevator Actual Postion", m_elevatorEncoder.getPosition());
   
    }
  }

}
