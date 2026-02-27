// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;



public class ShooterSubsystem extends SubsystemBase {


    private final SparkMax m_shooterShooterSpark;
    private final SparkMax m_shooterTurnerSpark;

    private final RelativeEncoder m_shooterShooterEncoder;
    private final AbsoluteEncoder m_shooterTurnerEncoder;

    private final SparkClosedLoopController m_shooterShooterClosedLoopController;
    private final SparkClosedLoopController m_shooterTurnerClosedLoopController;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_shooterShooterSpark = new SparkMax(Constants.ShooterConstants.kshooterShooterCANId, MotorType.kBrushless);
    m_shooterTurnerSpark = new SparkMax(Constants.ShooterConstants.kshooterTurnerCANId, MotorType.kBrushless);

    // Retrieve encoders and closed-loop controllers from the shooter sparks as local variables
    m_shooterShooterEncoder = m_shooterShooterSpark.getEncoder();
    m_shooterTurnerEncoder = m_shooterTurnerSpark.getAbsoluteEncoder();

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
  }

  public void setShooterSpeed(double shooterSpeed) {
    m_shooterShooterClosedLoopController.setReference(shooterSpeed, ControlType.kVelocity);
  }

  public void setTurnerAngle(double turnerAngle) {
    m_shooterTurnerClosedLoopController.setReference(turnerAngle, ControlType.kPosition);
  }

}
