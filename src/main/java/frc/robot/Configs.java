package frc.robot;

import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
       
        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

           
            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);

           
        }
    }

    public static final class Coral {

        public static final SparkMaxConfig troughConfig = new SparkMaxConfig();
        public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        public static final ServoHubConfig servoConfig = new ServoHubConfig();

        static {

            double troughRotationFactor = 2 * Math.PI;

            troughConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(false);

                troughConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(false)
                    .positionConversionFactor(troughRotationFactor) // radians
                    .velocityConversionFactor(troughRotationFactor / 60.0); // radians per second
                troughConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(CoralConstants.ktroughP, CoralConstants.ktroughI, CoralConstants.ktroughD)
                    .outputRange(-0.15, 0.15)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(false);
                    //.positionWrappingInputRange(0, troughRotationFactor);

            elevatorConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(true);

            elevatorConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(CoralConstants.ktroughP, CoralConstants.kelevtorI, CoralConstants.kelevtord)
               
                    .outputRange(-1, 1);

        
            servoConfig
                .channel0.pulseRange(500, 1500, 2500)     
                .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
                           
                

        }

    }
        public static final class Climber {

                public static final SparkMaxConfig leftwristConfig = new SparkMaxConfig();
                public static final SparkMaxConfig rightwristConfig = new SparkMaxConfig();
                public static final SparkMaxConfig climberConfig = new SparkMaxConfig();
        
                static {
        
                
        
                    leftwristConfig
                            .idleMode(IdleMode.kBrake)
                            .smartCurrentLimit(50)
                            .inverted(true);
        
                        
                            
                        leftwristConfig.closedLoop
                            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // These are example gains you may need to them for your own robot!
                            .pid(ClimberConstants.kwristP,ClimberConstants.kwristI,ClimberConstants.kwristD)
                            .outputRange(-.4, .4)
                            // Enable PID wrap around for the turning motor. This will allow the PID
                            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                            // to 10 degrees will go through 0 rather than the other direction which is a
                            // longer route.
                            .positionWrappingEnabled(false);
                          

                            
                    rightwristConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(false);

                
                    
                rightwristConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ClimberConstants.kwristP,ClimberConstants.kwristI,ClimberConstants.kwristD)
                    .outputRange(-.4, .4)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true); 


                    double climberRotationFactor = 2 * Math.PI;
                    climberConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(true);

                climberConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(climberRotationFactor) // radians
                    .velocityConversionFactor(climberRotationFactor / 60.0); // radians per second



                    
                climberConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ClimberConstants.kclimberP,ClimberConstants.kclimberI,ClimberConstants.kclimberD)
                    .outputRange(-.8, .8)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(false);
                  
            
                    
        
                
                        
                        
        
                }
    }








}
