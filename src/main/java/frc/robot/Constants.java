// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 5;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;

    // Toggle for field relative driving
    public static boolean driveFieldRelative = true;


  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }



  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class OIConstants {
    public static final double kDriveDeadband = 0.05;
    
    // Controller Ports
    public static final int kLeftControllerPort = 0;
    public static final int kRightControllerPort = 1;
    public static final int kButtonBoardPort = 2 ;


    // Left Controller Buttons
  
    public static final int kejectButton = 1; //Left Trigger to eject
    public static final int kSetXButton = 4; // Need to decide which stick and button we should use...
    public static final int kGyroRestButton = 5;  // Need to decide which stick and button we should use...






    // Right Controller Buttons
  public static final int kintakeButton = 1; //Right Trigger to intake
    public static final int kFieldRelativeButton = 3;


    // button board buttons
    public static final int kAutoAimButton = 1;
    public static final int kManualAimButton = 2;
    public static final int kShootButton = 3;
    //public static final int kL3EButton = 4;
    //public static final int kShoot = 5;
    public static final int kStowButton = 6;
    //public static final int kL1TButton = 7;
    //public static final int kL2TButton = 8;
    //public static final int kL3TButton = 9;
    //public static final int kL4TButton = 10;
    public static final int kleftArmUpButton = 11;
    public static final int kleftArmDownButton = 12;
    //  public static final int  = 13;
    // public static final int  = 14;
    public static final int krightArmUpButton = 15;
    public static final int krightArmDownButton = 16;
    public static final int kshooterInfoButton = 22;
    public static final int kClimberInfoButton = 23;
    public static final int kdriveInfoButton = 24; // Moved from left stick
  

  }

  public static final class IntakeConstants {

    //  constants for the intake subsystem

    public static final int klowerIntakeCANId = 13; 
    public static final double klowerIntakeIntakePower = 0.05;
    public static final double klowerIntakeEjectPower = -0.05;
    public static final double klowerIntakeShootPower = 0.1;


    public static final int kupperIntakeCANId = 10;
    public static final double kupperIntakeIntakePower = 0.3;
    public static final double kupperIntakeEjectPower = 0.3;
    public static final double kupperIntakeShootPower = 0.3; 

  }
 
  public static final class ClimberConstants {

    //  constants for thee climber subsystem

    public static final int kleftClimberCANId = 11; 
    public static final double kleftWristPower = 0.15;
   
    public static final int krightClimberCANId = 9; 
    public static final double krightWristPower = 0.15;
   
    public static final int kclimberCANId = 12;
    public static final double kclimberPower =0.3;
    
    public static final double kclimberP = 0.3;
    public static final double kclimberI = 0.0;
    public static final double kclimberD = 0.0;



    public static final double krightArmUp = 1.0;
    public static final double krightArmDown = 0.0;
    public static final double kleftArmUp =1.0; 
    public static final double kleftArmDown = 0.0;
  
  }


  public static final class ShooterConstants {

    //  constants for the shooter subsystem

    public static final int kshooterShooterCANId = 14; 
    public static final double kshooterShooterSpeed = 0.3;
   
    public static final int kshooterTurnerCANId = 15; 
    public static final double kshooterturnerhomePostion = 0.0;
   
    
    public static final double kShooterP = 0.3;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;

    public static final double kShooterTurnerP = 0.3;
    public static final double kShooterTurnerI = 0.0;
    public static final double kShooterTurnerD = 0.0;
  }

}
