// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.concurrent.RunnableFuture;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
    Optional<Alliance> ally = DriverStation.getAlliance();
    private String alli = "None! (What's up with that?)";
    String autonStatus = "Not Trying Anything";


  
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final VisionSubsystem m_vision = new VisionSubsystem(m_robotDrive);
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ClimberSubsystem m_climb = new ClimberSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final AutoFactory autoFactory;

    private final Timer timer = new Timer();
    

 
    // The driver's controllers
    Joystick m_leftJoystick = new Joystick(OIConstants.kLeftControllerPort);
    Joystick m_rightJoystick = new Joystick(OIConstants.kRightControllerPort);
    Joystick m_buttonboard = new Joystick(OIConstants.kButtonBoardPort);

 

 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    autoFactory = new AutoFactory(
           m_robotDrive::getPose, // A function that returns the current robot pose
           m_robotDrive::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
           m_robotDrive::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            m_robotDrive // The drive subsystem
        );


    
    // Run configuration options for Pigeon 2 navigation module
    m_robotDrive.pidgeyConfig();

    // Confirm Turret Encoder is zeroed at startup to prevent runaway turret
    m_shooter.zeroTurretEncoder();


    // Configure the button bindings
    configureButtonBindings();

     //  Configure dashboard
    configureDashboard();
       
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
       // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_leftJoystick.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_leftJoystick.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_rightJoystick.getZ(), OIConstants.kDriveDeadband),
                DriveConstants.driveFieldRelative),
            m_robotDrive)); 





  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //  Set Wheels in an X configuration to prevent movement
    new JoystickButton(m_leftJoystick, OIConstants.kSetXButton)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    //  Zero the gyro to prevent drift
    new JoystickButton(m_leftJoystick, OIConstants.kGyroRestButton)
        .debounce(0.1)   
        .whileTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    //  Toggle Field Centric vs Robot Centric Driving
    new JoystickButton(m_rightJoystick, OIConstants.kFieldRelativeButton)
        .debounce(0.1)   
        .whileTrue(new InstantCommand(
            () -> m_robotDrive.toggleFieldRelative(),
            m_robotDrive));

        

    //  Intake on
    new JoystickButton(m_rightJoystick, OIConstants.kintakeButton)
    .debounce(0.1)   
    .whileTrue(new InstantCommand(
        () -> m_intake.setIntake(IntakeConstants.klowerIntakeIntakePower,IntakeConstants.kupperIntakeIntakePower),
        m_intake))
    .whileFalse(new InstantCommand(
        () -> m_intake.setIntake(0,0),
        m_intake));
        
        
    

    //  Eject
    new JoystickButton(m_leftJoystick, OIConstants.kejectButton)
    .debounce(0.1)   
    .whileTrue(new InstantCommand(
        () -> m_intake.setIntake(IntakeConstants.klowerIntakeEjectPower,IntakeConstants.kupperIntakeEjectPower),
        m_intake))
    .whileFalse(new InstantCommand(
        () -> m_intake.setIntake(0,0),
        m_intake));
      
    //****************** Shooter Stuff ***********************         

    //  Shoot
    new JoystickButton(m_buttonboard, OIConstants.kShootButton)
    .debounce(0.1)   
    .whileTrue(new InstantCommand(
        () -> m_shooter.setShooterSpeed(ShooterConstants.kshooterShooterSpeed),
        m_shooter))
    .whileTrue(new InstantCommand(
        () -> m_intake.setIntake(IntakeConstants.klowerIntakeShootPower,IntakeConstants.kupperIntakeShootPower),
        m_intake))
    .whileFalse(new InstantCommand(
        () -> m_intake.setIntake(0,0),
        m_intake));
      

    //  Shooter off
    new JoystickButton(m_buttonboard, OIConstants.kShooterOffButton)
    .debounce(0.1)   
    .whileTrue(new InstantCommand(
        () -> m_shooter.stopShooter(),
        m_shooter));

       

    //  Stow shooter
    new JoystickButton (m_buttonboard,OIConstants.kStowButton)
    .onTrue(
        new InstantCommand (
        () -> m_shooter.setTurnerAngle(ShooterConstants.kshooterturnerhomePostion),
        m_shooter)
        );


    //  Turn on manual shooter control
    new JoystickButton (m_buttonboard,OIConstants.kManualAimButton)
    .whileTrue(
        new RunCommand(
            () -> m_shooter.setManualTurretPower(-0.1*m_buttonboard.getZ()), // Adjust the angle increment as needed
            m_shooter))
    .whileFalse(new InstantCommand(
        () -> m_shooter.setManualTurretPower(0),
        m_shooter));

    // Auto-aim at a fixed field pose while the AutoAim button is held
    new JoystickButton(m_buttonboard, OIConstants.kAutoAimButton)
        .whileTrue(
         new RunCommand(() -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_leftJoystick.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_leftJoystick.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_vision.aimAtFieldLocation(m_robotDrive))*VisionConstants.VISION_TURN_kP, OIConstants.kDriveDeadband),
                DriveConstants.driveFieldRelative),
            m_robotDrive));       
         
    //  Shooter Speed Up
    new JoystickButton(m_leftJoystick, OIConstants.kshooterSpeedUpButton)
        .onTrue(new InstantCommand(
            () -> m_shooter.shooterSpeedUp(),
            m_shooter)
            
        );

        //  Shooter Speed down
    new JoystickButton(m_leftJoystick, OIConstants.kshooterSpeedDownButton)
        .onTrue(new InstantCommand(
            () -> m_shooter.shooterSpeedDown(),
            m_shooter)
            
        );

    
    

    //////////////////////////////////  Shuffleboard Toggles  /////////////////////////////////////////

    //  Toggle Extra Info to Shuffleboard
    new JoystickButton(m_buttonboard, OIConstants.kdriveInfoButton)
        .whileTrue(new InstantCommand(
            () -> m_robotDrive.toggleDriveDebugInfo(),
            m_robotDrive));


    //  Toggle Climber Info to Shuffleboard
    new JoystickButton(m_buttonboard, OIConstants.kClimberInfoButton)
    .whileTrue(new InstantCommand(
        () -> m_climb.toggleClimberDebugInfo(),
        m_climb));
 

    // Toggle Shooter Debug Info to Shuffleboard
    new JoystickButton(m_buttonboard, OIConstants.kshooterInfoButton)
        .whileTrue(new InstantCommand(
            () -> m_shooter.toggleShooterDebugInfo(),
            m_shooter));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////



    // ********************Arm Stuff**************************

    //  Left Arm Up
    new JoystickButton (m_buttonboard,OIConstants.kleftArmUpButton)
    .whileTrue( 
        new InstantCommand(
        () -> m_climb.setLeftClimber(ClimberConstants.kleftArmUp),
        m_climb));

    //  Left Arm Down
    new JoystickButton (m_buttonboard,OIConstants.kleftArmDownButton)
    .whileTrue( 
        new InstantCommand(
        () -> m_climb.setLeftClimber(ClimberConstants.kleftArmDown),
        m_climb));
  
    //  Right Arm Up
    new JoystickButton (m_buttonboard,OIConstants.krightArmUpButton)
    .whileTrue( 
        new InstantCommand(
        () -> m_climb.setRightClimber(ClimberConstants.krightArmUp),
        m_climb));

    //  Right Arm Down
    new JoystickButton (m_buttonboard,OIConstants.krightArmDownButton)
    .whileTrue( 
        new InstantCommand(
        () ->    m_climb.setRightClimber(ClimberConstants.krightArmDown),
        m_climb));
  }

  private void configureDashboard() {
        
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) { alli="Red";}
        if (ally.get() == Alliance.Blue) { alli="Blue";}
    }
  
    SmartDashboard.putString(   "Alliance", alli);
    SmartDashboard.putString("Attempted Auton", autonStatus);
    
    
    // Create the auto chooser
    AutoChooser autoChooser = new AutoChooser();

    // Add options to the chooser
    autoChooser.addCmd("Left Shoot n Move", this::leftShootMoveCommand);
    autoChooser.addCmd("Left Shoot n Move", this::leftShootStayCommand);
    autoChooser.addCmd("Right Shoot n Move", this::rightShootMoveCommand);
    autoChooser.addCmd("Right Shoot n Move", this::rightShootStayCommand);
    autoChooser.addCmd("Center Shoot n Move", this::centerShootMoveCommand);
    autoChooser.addCmd("Center Shoot n Move", this::centerShootMoveCommand);


    // Put the auto chooser on the dashboard
    SmartDashboard.putData("Options",autoChooser);


    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    
    
    // Log Shuffleboard events for command initialize, execute, finish, interrupt
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command ->
                Shuffleboard.addEventMarker(
                    "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandExecute(
            command ->
                Shuffleboard.addEventMarker(
                    "Command executed", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command ->
                Shuffleboard.addEventMarker(
                    "Command finished", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command ->
                Shuffleboard.addEventMarker(
                    "Command interrupted", command.getName(), EventImportance.kNormal));
    
  }




    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Command leftShootMoveCommand() {
        autonStatus = "Left Wall ShootnMove";
        SmartDashboard.putString("Attempting", autonStatus);

        return Commands.sequence(
            // Set shooter speed once
            
            new InstantCommand(
                () -> m_shooter.setShooterSpeed(ShooterConstants.kshooterShooterSpeed),
                m_shooter
            ),


            new InstantCommand(
                () -> m_shooter.setTurnerAngle(15.0),
                    m_shooter
            ),

            // Wait for shooter to spin up
            Commands.waitSeconds(2),
             
            // Start intake (one-shot to set motors)
            new InstantCommand(
                () -> m_intake.setIntake(IntakeConstants.klowerIntakeShootPower, IntakeConstants.kupperIntakeShootPower),
                m_intake
            ),
        
            // Wait for shots
            Commands.waitSeconds(10.0),

            // Drive forward for 1 second while intake/shooter are running
            Commands.run(() -> m_robotDrive.drive(-0.10, 0.0, 0.0,true), m_robotDrive).withTimeout(2.5),

            // Stop drive and intake when done
            new InstantCommand(
                () -> {
                    m_robotDrive.drive(0.0, 0.0, 0.0,true);
                    m_intake.setIntake(0, 0);
                    m_shooter.turretOff();
                },
                m_robotDrive,
                m_intake,
                m_shooter
            )
        );
    }/////////////////////////////////////////////////////////////////////////////////////////////


        public Command leftShootStayCommand() {
        autonStatus = "Left Wall ShootnStay";
        SmartDashboard.putString("Attempting", autonStatus);

        return Commands.sequence(
            // Set shooter speed once
            new InstantCommand(
                () -> m_shooter.setShooterSpeed(ShooterConstants.kshooterShooterSpeed),
                m_shooter
            ),

            // Wait for shooter to spin up
            Commands.waitSeconds(2),

            
             
            // Start intake (one-shot to set motors)
            new InstantCommand(
                () -> m_intake.setIntake(IntakeConstants.klowerIntakeShootPower, IntakeConstants.kupperIntakeShootPower),
                m_intake
            ),
        

            // Wait for shots
            Commands.waitSeconds(10.0),

            // Drive forward for 1 second while intake/shooter are running
            Commands.run(() -> m_robotDrive.drive(-0.09, 0.0, 0.0,true), m_robotDrive).withTimeout(1.0),

            // Stop drive and intake when done
            new InstantCommand(
                () -> {
                    m_robotDrive.drive(0.0, 0.0, 0.0,true);
                    m_intake.setIntake(0, 0);
                },
                m_robotDrive,
                m_intake
            )
        );
    }/////////////////////////////////////////////////////////////////////////////////////////////



    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Command rightShootMoveCommand() {
        autonStatus = "Right Shoot n Move";
        SmartDashboard.putString("Attempting", autonStatus);

    return Commands.sequence(
            autoFactory.resetOdometry("RightSide"), // 

            Commands.deadline(
                autoFactory.trajectoryCmd("RightSide") //
                // 
                ),
            
                // Set shooter speed once
                new InstantCommand(
                    () -> m_shooter.setShooterSpeed(ShooterConstants.kshooterShooterSpeed),
                    m_shooter
                ),

                


                new InstantCommand(
                    () -> m_shooter.setTurnerAngle(50.0),
                    m_shooter
                ),

                // Wait for shooter to spin up and turret to rotate
                Commands.waitSeconds(2),
                
                // Start intake (one-shot to set motors)
                new InstantCommand(
                    () -> m_intake.setIntake(IntakeConstants.klowerIntakeShootPower, IntakeConstants.kupperIntakeShootPower),
                    m_intake
                ),
            

                // Wait for shots
                Commands.waitSeconds(10.0),

                // Drive forward for 1 second while intake/shooter are running
                //Commands.run(() -> m_robotDrive.drive(-0.09, 0.0, 0.0,true), m_robotDrive).withTimeout(1.0),

                // Stop drive and intake when done
                new InstantCommand(
                    () -> {
                        m_robotDrive.drive(0.0, 0.0, 0.0,true);
                        m_intake.setIntake(0, 0);
                        m_shooter.setTurnerAngle(0);
                       
                    },
                    m_robotDrive,
                    m_intake,
                    m_shooter
                )
            );
        
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////




        //////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Command rightShootStayCommand() {
        autonStatus = "Right Shoot n Stay";
        SmartDashboard.putString("Attempting", autonStatus);

    return Commands.sequence(
                // Set shooter speed once
                new InstantCommand(
                    () -> m_shooter.setShooterSpeed(ShooterConstants.kshooterShooterSpeed),
                    m_shooter
                ),

                new InstantCommand(
                    () -> m_shooter.setTurnerAngle(185),
                    m_shooter
                ),

                // Wait for shooter to spin up and turret to rotate
                Commands.waitSeconds(2),
              

                
                
                // Start intake (one-shot to set motors)
                new InstantCommand(
                    () -> m_intake.setIntake(IntakeConstants.klowerIntakeShootPower, IntakeConstants.kupperIntakeShootPower),
                    m_intake
                ),
            

                // Wait for shots
                Commands.waitSeconds(10.0),

              
                // Stop drive and intake when done
                new InstantCommand(
                    () -> m_intake.setIntake(0.0, 0.0),
                    m_intake
                )
            );
        
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
   
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Command centerShootMoveCommand() {
        autonStatus = "Center Shoot n Move";
        SmartDashboard.putString("Attempting", autonStatus);

    return Commands.sequence(
                // Set shooter speed once
                new InstantCommand(
                    () -> m_shooter.setShooterSpeed(ShooterConstants.kbloopShooterSpeed),
                    m_shooter
                ),
                
                // Drive forward for 1 second while intake/shooter are running
                Commands.run(
                    () -> m_robotDrive.drive(-0.12, 0.0, 0.0,true), m_robotDrive).withTimeout(3.0),

                


                new InstantCommand(
                    () -> m_shooter.setTurnerAngle(98.0),
                    m_shooter
                ),

       
                // Wait robot to move for shooter to spin up and turret to rotate
                Commands.waitSeconds(5),
                
                // Start intake (one-shot to set motors)
                new InstantCommand(
                    () -> m_intake.setIntake(IntakeConstants.klowerIntakeShootPower, IntakeConstants.kupperIntakeShootPower),
                    m_intake
                ),
             
                // Drive forward for 1 second while intake/shooter are running
                Commands.run(
                    () -> m_robotDrive.drive(0.0, 0.0, 0.0,true), m_robotDrive).withTimeout(2.5),            

                // Wait for shots
                Commands.waitSeconds(10.0),

                // Stop drive and intake when done
                new InstantCommand(
                    () -> {
                        m_robotDrive.drive(0.0, 0.0, 0.0,true);
                        m_intake.setIntake(0, 0);
                        m_shooter.turretOff();
                    },
                    m_robotDrive,
                    m_intake,
                    m_shooter
                )
            );
        
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////




        //////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Command centerShootStayCommand() {
        autonStatus = "Center Shoot n Stay";
        SmartDashboard.putString("Attempting", autonStatus);

    return Commands.sequence(
                // Set shooter speed once
                new InstantCommand(
                    () -> m_shooter.setShooterSpeed(ShooterConstants.kbloopShooterSpeed),
                    m_shooter
                ),

                new InstantCommand(
                    () -> m_shooter.setTurnerAngle(98),
                    m_shooter
                ),

                // Wait for shooter to spin up and turret to rotate
                Commands.waitSeconds(2),
              

                
                
                // Start intake (one-shot to set motors)
                new InstantCommand(
                    () -> m_intake.setIntake(IntakeConstants.klowerIntakeShootPower, IntakeConstants.kupperIntakeShootPower),
                    m_intake
                ),
            

                // Wait for shots
                Commands.waitSeconds(10.0),

              
                // Stop drive and intake when done
                new InstantCommand(
                    () -> m_intake.setIntake(0.0, 0.0),
                    m_intake
                )
            );
        



    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////    




}
