// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
            false, // If alliance flipping should be enabled 
            m_robotDrive // The drive subsystem
        );
    
    
    // Run configuration options for Pigeon 2 navigation module
    m_robotDrive.pidgeyConfig();


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
    .debounce(0.03)   
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

       


    // Auto-aim at a fixed field pose while the AutoAim button is held
    new JoystickButton(m_buttonboard, OIConstants.kAutoAimButton)
        .whileTrue(
            new RunCommand(() -> {
                double[] powerPlusRange = m_vision.aimAtFieldLocation(m_robotDrive);
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_leftJoystick.getY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_leftJoystick.getX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(powerPlusRange[0], OIConstants.kDriveDeadband),
                    DriveConstants.driveFieldRelative);
                m_shooter.setShooterSpeedFromRange(powerPlusRange[1]);
            }, m_vision, m_robotDrive, m_shooter));



            
    // Auto-aim at a fixed field pose while the AutoAim button is held
    new JoystickButton(m_buttonboard, OIConstants.kPointAtHubButton)
        .whileTrue(
         new RunCommand(() -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_leftJoystick.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_leftJoystick.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_vision.twistPower, OIConstants.kDriveDeadband),
                DriveConstants.driveFieldRelative),
            m_robotDrive));  






         
    //  Shooter Speed Up
    new JoystickButton(m_buttonboard, OIConstants.kshooterSpeedUpButton)
        .onTrue(new InstantCommand(
            () -> m_shooter.shooterSpeedUp(),
            m_shooter)
            
        );

        //  Shooter Speed down
    new JoystickButton(m_buttonboard, OIConstants.kshooterSpeedDownButton)
        .onTrue(new InstantCommand(
            () -> m_shooter.shooterSpeedDown(),
            m_shooter)
            
        );

    
    

    //////////////////////////////////  Shuffleboard Toggles  /////////////////////////////////////////


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

    //  Arms up
    new JoystickButton (m_buttonboard,OIConstants.karmsUpButton)
    .whileTrue( 
        new InstantCommand(
        () -> {
            m_climb.setBlackArm(ClimberConstants.kblackArmUp);
            m_climb.setOrangeArm(ClimberConstants.korangeArmUp);
        },
        m_climb));

    //  Orange Arm Up
    new JoystickButton (m_buttonboard,OIConstants.korangeArmUpButton)
    .whileTrue( 
        new InstantCommand(
        () -> m_climb.setOrangeArm(ClimberConstants.korangeArmUp),
        m_climb));

     //  Orange Arm Down
     new JoystickButton (m_buttonboard,OIConstants.korangeArmDownButton)
     .whileTrue( 
         new InstantCommand(
         () -> m_climb.setOrangeArm(-ClimberConstants.korangeArmDown),
         m_climb));

     //  Black Arm Up
     new JoystickButton (m_buttonboard,OIConstants.kblackArmUpButton)
     .whileTrue( 
         new InstantCommand(
         () -> m_climb.setBlackArm(ClimberConstants.kblackArmUp),
         m_climb));

      //  Black Arm Down
      new JoystickButton (m_buttonboard,OIConstants.kblackArmDownButton)
      .whileTrue( 
          new InstantCommand(
          () -> m_climb.setBlackArm(-ClimberConstants.kblackArmDown),
          m_climb));    

    //  Black Climber Up
    new JoystickButton (m_buttonboard,OIConstants.kblackClimberUpButton)
    .whileTrue( 
        new InstantCommand(
        () -> m_climb.setBlackClimber(ClimberConstants.kblackClimberUp),
        m_climb));

    //  Black Climber Down
    new JoystickButton (m_buttonboard,OIConstants.kblackaClimberDownButton)
    .whileTrue( 
        new InstantCommand(
        () -> m_climb.setBlackClimber(ClimberConstants.kblackClimberDown),
        m_climb));
  
    //  Orange Climber Up
    new JoystickButton (m_buttonboard,OIConstants.korangeClimberUpButton)
    .whileTrue( 
        new InstantCommand(
        () -> m_climb.setOrangeClimber(ClimberConstants.korangeClimberUp),
        m_climb));

    //  Orange Climber Down
    new JoystickButton (m_buttonboard,OIConstants.korangeClimberDownButton)
    .whileTrue( 
        new InstantCommand(
        () ->    m_climb.setOrangeClimber(ClimberConstants.korangeClimberDown),
        m_climb));

    //  Zero Climber
    new JoystickButton (m_buttonboard,OIConstants.kzeroClimberButton)
    .onTrue( 
        new InstantCommand(
        () -> m_climb.zeroClimber(),
        m_climb));

    //  Climb! Combinded Command
    new JoystickButton (m_buttonboard,OIConstants.kclimberdownButton)
    .onTrue( 
        new InstantCommand(
        () -> m_climb.climbCombined(),
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
    //autoChooser.addCmd("Left Move Shoot", this::leftMoveShootCommand);
    autoChooser.addCmd("Basic Backup", this::basicBackupCommand);
    autoChooser.addCmd("REV Example", this::revAutonomousCommand);
    //autoChooser.addCmd("Right Shoot n Move", this::rightShootStayCommand);
    //autoChooser.addCmd("Center Shoot n Move", this::centerShootMoveCommand);
    //autoChooser.addCmd("Center Shoot n Move", this::centerShootMoveCommand);


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

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Command basicBackupCommand() {
        autonStatus = "Basic Backup Command";
        SmartDashboard.putString("Attempting", autonStatus);
        return Commands.sequence(
            //Commands.run(() -> m_robotDrive.drive(-10.0,0,0,false)).withTimeout(3),
            new InstantCommand(() -> m_shooter.setShooterSpeed(ShooterConstants.kshooterAutonShootSpeed), m_shooter),
            Commands.waitSeconds(2.0) .andThen(
                new InstantCommand(() -> m_intake.setIntake(IntakeConstants.klowerIntakeShootPower, IntakeConstants.kupperIntakeShootPower),
                    m_intake))

            //Commands.waitSeconds(5),
            //Commands.runOnce(() -> m_robotDrive.drive(0, 0, 0, true)).withTimeout(2)
        );
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 


    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Command leftMoveShootCommand() {
        autonStatus = "Left Wall Move Then Shoot";
        SmartDashboard.putString("Attempting", autonStatus);

        return Commands.sequence(
            
          
            autoFactory.resetOdometry("left2Shoot"), 
            Commands.parallel(
                autoFactory.trajectoryCmd("left2Shoot")   /* ,
             
                new InstantCommand(() -> m_shooter.setShooterSpeed(ShooterConstants.kshooterAutonShootSpeed), m_shooter),
                Commands.waitSeconds(2.0),
                new InstantCommand(() -> m_intake.setIntake(IntakeConstants.klowerIntakeShootPower, IntakeConstants.kupperIntakeShootPower),
                    m_intake)
             */            
        ));
        
        
       
    }/////////////////////////////////////////////////////////////////////////////////////////////



    ////////////////    REV Basic Auto    ////////////////////////////////////////////////////////////////////////////////////////
   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command revAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1, 0), new Translation2d(-1.5, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-1.9558, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(
        () -> m_robotDrive.drive(0, 0, 0, false))
        .andThen(
        new InstantCommand(() -> m_shooter.setShooterSpeed(ShooterConstants.kshooterAutonShootSpeed), m_shooter),
        Commands.waitSeconds(2.0) .andThen(
        new InstantCommand(() -> m_intake.setIntake(IntakeConstants.klowerIntakeShootPower, IntakeConstants.kupperIntakeShootPower),
                    m_intake)));
  }




}
