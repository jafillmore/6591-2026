// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
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

    UsbCamera camera1;
    VideoSink server;
    
  
    // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ClimberSubsystem m_climb = new ClimberSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final AutoFactory autoFactory;

  private final Timer timer = new Timer();


  // Loads a swerve trajectory, alternatively use DifferentialSample if the robot is tank drive
  private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("startonwall");


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

    

    camera1 = CameraServer.startAutomaticCapture(0);
    
    server = CameraServer.getServer();
    
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

    //  Toggle Climber Info to Shuffleboard
    new JoystickButton(m_buttonboard, OIConstants.kClimberInfoButton)
    .whileTrue(new InstantCommand(
        () -> m_climb.toggleClimberDebugInfo(),
        m_climb));
 
    //  Toggle Drive Info to Shuffleboard
    new JoystickButton(m_leftJoystick, OIConstants.kdriveInfoButton)
    .whileTrue(new InstantCommand(
        () -> m_robotDrive.toggleDriveDebugInfo(),
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
        () -> m_shooter.setShooterSpeed(0),
        m_shooter))
    .whileFalse(new InstantCommand(
        () -> m_intake.setIntake(0,0),
        m_intake));
      


    //  Toggle Extra Info to Shuffleboard
    new JoystickButton(m_leftJoystick, OIConstants.kdriveInfoButton)
        .whileTrue(new InstantCommand(
            () -> m_robotDrive.toggleDriveDebugInfo(),
            m_robotDrive));

    //  Stow shooter
    new JoystickButton (m_buttonboard,OIConstants.kStowButton)
    .onTrue(
        new InstantCommand (
        () -> m_shooter.setTurnerAngle(ShooterConstants.kshooterturnerhomePostion),
        m_shooter)
        );



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


   ///////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Initiation items for Auton
    /// ///////////////////////////////////////////////////////////////////////////////////////////////////

    public void autoInit() {
        
        if (trajectory.isPresent()) {
            // Get the initial pose of the trajectory
            Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());
            SmartDashboard.putBoolean(   "Traj Loaded",trajectory.isPresent());

            if (initialPose.isPresent()) {
                // Reset odometry to the start of the trajectory
                m_robotDrive.resetOdometry(initialPose.get());
                SmartDashboard.putBoolean(   "Initial Pose",initialPose.isPresent());

            }
        }

        // Reset and start the timer when the autonomous period begins
        timer.restart();
        SmartDashboard.putNumber("Time", timer.get());

    }


    ///////////////////////////////////////////////////////////////////////////////
    private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }
    ///////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    //////////////////////////////////////////////////////////////////////////
    public Command getAutonomousCommand() {
        
        return Commands.sequence(
            autoFactory.resetOdometry("startonwall"), // 
            Commands.deadline(
                autoFactory.trajectoryCmd("startonwall")
                
        ));
    }

    /////////////////////////////////////////////////////////////////////////
    /// Auto Routine to move to the reef, clear algae and score coral/
    /// /////////////////////////////////////////////////////////////////////

    

    public AutoRoutine goToReefAuto() {
        AutoRoutine routine = autoFactory.newRoutine("scoreOnReef");

        // Load the routine's trajectories
        AutoTrajectory scoreOnReefTraj = routine.trajectory("startonwall");

        // When the routine begins, reset odometry and start the first trajectory (1)
        goToReefAuto().active().onTrue(
            Commands.sequence(
                scoreOnReefTraj.resetOdometry(),
                scoreOnReefTraj.cmd()
            )
        );

        

        return routine;
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////////
    /// Auton periodic for non-autofactory control
   
    public void autoPeriotic() {
     
        if (trajectory.isPresent()) {
            // Sample the trajectory at the current time into the autonomous period
           Optional<SwerveSample>   sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

            if (sample.isPresent()) {
               SmartDashboard.putBoolean(   "Sample is Present", sample.isPresent());
               //SmartDashboard.putBoolean(   "Routine Active", goToReefAuto().active().getAsBoolean());
               SmartDashboard.putNumber("Vx", sample.get().vx);
               SmartDashboard.putNumber("Vy", sample.get().vy);
               SmartDashboard.putNumber("Omega", sample.get().omega);
            
                m_robotDrive.followTrajectory(sample.get());
        
            }
        }
        
    }



}
