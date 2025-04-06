package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import com.revrobotics.spark.config.*;

public class ToolSubsystems {

    //getting all the motors from the robot container
    public  SparkMax algealeft = RobotContainer.algaeleft;
    public  SparkMax algearight = RobotContainer.algaeright;
    public  SparkMax pivot = RobotContainer.pivot;
    public  TalonFX elevator = RobotContainer.elevatorMotor;
    public  SparkMax coral = RobotContainer.coral;
    public  TalonFX lift = RobotContainer.lift;

    //setting the SparkMax variables, for motion magic, which allows for position control
    private  SparkMaxConfig config = new SparkMaxConfig();
    private  SparkClosedLoopController pivotSparkPID; 

    //Setting the Talon Fx variables and libraries, live velocity voltage, which allows for position control
    private  MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
    public VelocityVoltage m_velocity = new VelocityVoltage(0);
    public TalonFXConfiguration talonFxConfigs= new TalonFXConfiguration();
    private MotionMagicConfigs motionMagicConfigs = talonFxConfigs.MotionMagic;
    private Slot0Configs pids =  talonFxConfigs.Slot0;   
    private double elevatorPosition = 0; 

    //string to get pivot state
    String  pivotState = "default";


    //intiation commands
    public void init()
    {
            //setting encoder velocity settings to 0
            m_velocity.Slot = 0;
            //configuring talon fx motor
            elevator.getConfigurator().apply(talonFxConfigs, 0.50);
    }

    //sets all the configs for the motors for position control
    public void SetConfig()
    {
        //elevator / talonfx pids 
        pids.kS = 0.24;
        pids.kV = 0.12;
        pids.kP = 4.8;
        pids.kI = 0;
        pids.kD = 0.1;  
        
        //motion magic for elevator encoder pids
        motionMagicConfigs.MotionMagicCruiseVelocity = 90;
        motionMagicConfigs.MotionMagicAcceleration = 180;
        motionMagicConfigs.MotionMagicJerk = 1600;
        
        //from here on, this is all sparkmax initialization for encoder position setting
        //getting closed loop control aka encoder 
        pivotSparkPID = pivot.getClosedLoopController();
        config.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)

        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
            
        //creating library that has code to let us use the encoder as a position setter, motion max for sparkmax
        m_motmag.Slot = 0;
        
        //telling motor to give resistance when robot is on and its velocity is at 0
        config.idleMode(IdleMode.kBrake);
        
        SparkMaxConfig algeaConfig = config;
        algeaConfig.smartCurrentLimit(25);
        config.smartCurrentLimit(45);

        var talonFXLiftConfig = lift.getConfigurator();
        var talonFXElevatorConfig = elevator.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();
        var liftLimitConfigs = new CurrentLimitsConfigs();

        // enable stator current limit to not blow out motors
        limitConfigs.StatorCurrentLimit = 75;
        liftLimitConfigs.StatorCurrentLimit = 55;
        limitConfigs.StatorCurrentLimitEnable = true;

        //apply said configs
        talonFXLiftConfig.apply(liftLimitConfigs);
        talonFXElevatorConfig.apply(limitConfigs);
        
        //setting configurations on all sparkmax motors
        algealeft.configure(algeaConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algearight.configure(algeaConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivot.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        coral.configure(algeaConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void LiftMotorForward()
    {
        lift.set(1);
    }

    public void LiftMotorBackwards()
    {
        lift.set(-1);
    }

    public void StopLiftMotor()
    {
        lift.set(0);
    }
    //function to manually move any sparkmax motor forward, for resets during game and stuff
    public void MoveMotorForwards(SparkMax motor)
    {
        motor.set(.75);
    }

    //function to manually move motor backwards,again for resets during game
    public void MoveMotorBackwards(SparkMax motor)
    {
        motor.set(-.75);
    }
    
    //function to manually stop motor, for resests during game and stuff
    public void StopMotor(SparkMax motor)
    {
        motor.set(0.0);
    }
    
    public void StartElevatorMotor()
    {
        elevator.set(.3);
    }
    
    public void StopElevatorMotor()
    {
        elevator.set(0);
    }

    public void BackElevatorMotor()
    {
        elevator.set(-0.3);
    }

    //sets the elevator motor to a certain position
    public void MoveElevatorMotor(double position)
    {
        elevator.setControl(m_motmag.withPosition(position));
        elevatorPosition = position;
    }


    //moves the algae motors to intake or outake
    public void MoveAlgea(boolean forward)
    {
        
        if(forward == true)
        {
            MoveMotorForwards(algealeft);
            MoveMotorBackwards(algearight);
        } else 
        {
            MoveMotorForwards(algearight);
            MoveMotorBackwards(algealeft);
        }
    }

    //stops the algea motors
    public void StopAlgae()
    {
        // algealeft.stopMotor();
        // algearight.stopMotor();
        algealeft.set(0);
        algearight.set(0);
    }

    //brings elevator to 0 and pivot to default to reset robot 
    public void ResetRobotParts()
    {
        PivotDefault();
        MoveElevatorMotor(0);
    }


    public void ResetElevatorEncoder()
    {
        elevator.setPosition(0);
    }
    
    public void moveUpABit() {
        elevator.setControl(m_motmag.withPosition(elevatorPosition + 1));
        elevatorPosition = elevatorPosition+1;
        Timer.delay(0.1);
    } //controls to help jude if encoder values mess up

    public void moveDownABit() {
        elevator.setControl(m_motmag.withPosition(elevatorPosition - 1));
        elevatorPosition = elevatorPosition - 1;
        Timer.delay(0.1);
    }  //controls to help jude if encoder values mess up

    public void StopElevator()
    {
        elevator.setControl(m_motmag.withPosition(elevatorPosition));
    } 

    
     // Funciton to set pivot position to orgin/ 0
     public void PivotDefault()
     {
         if(pivotState == "default")
         {
 
         }
 
         if(pivotState == "intake")
         {
         pivotSparkPID.setReference(-1, ControlType.kPosition, ClosedLoopSlot.kSlot0);
         Timer.delay(.1);
         pivotSparkPID.setReference(0 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
         }
 
         if(pivotState == "outake")
         {
             pivotSparkPID.setReference(-6 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
             Timer.delay(.2);
             pivotSparkPID.setReference(-4.5 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
             Timer.delay(.2);
             pivotSparkPID.setReference(-3.25 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
             Timer.delay(.2);
             pivotSparkPID.setReference(-2 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
             Timer.delay(.2);
             pivotSparkPID.setReference(0 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
         }
         pivotState = "default";
         
     }
         
     //function sets the pivot to the outtake position, taking a break in between positions to not fling out the coral
     public void PivotOutTake()
     {   
         if(pivotState == "default") //path to take if its starting in default
         {
         pivotSparkPID.setReference(0 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
         Timer.delay(.2);
         pivotSparkPID.setReference(-2 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
         Timer.delay(.2);
         pivotSparkPID.setReference(-3.25 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
         Timer.delay(.2);
         pivotSparkPID.setReference(-4.5 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
         Timer.delay(.2);
         pivotSparkPID.setReference(-6 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
         } 
 
         if(pivotState == "intake") // path to take if it starts in intake
         {
          pivotSparkPID.setReference(-2 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
         Timer.delay(.2);
         pivotSparkPID.setReference(-3.25 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
         Timer.delay(.2);
         pivotSparkPID.setReference(-4.5 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
         Timer.delay(.2);
         pivotSparkPID.setReference(-6 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
         }
 
         if(pivotState == "outake") // nothing if its already in outake.
         {
 
         }
         pivotState = "outake"; //sets current state to outake
     }
 
     //position to intake the coral
     public void PivotIntake()
     {
         pivotSparkPID.setReference(-1.2 , ControlType.kPosition, ClosedLoopSlot.kSlot0);
         pivotState = "intake"; // dont need states for the intake state
     }

    
    public void FlipMaxSpeed()
    {
        RobotContainer.MaxSpeed = -RobotContainer.MaxSpeed;
        System.out.println("Control Speed is now " + RobotContainer.MaxSpeed);
    }

    //configs for switching to red side, invert controls and auto and such
    public void redSideConfig()
    {
        RobotContainer.MaxSpeed = -RobotContainer.MaxBaseSpeed; //inverts speed 
        RobotContainer.autoLineUpSpeed = RobotContainer.baseautoLineUpSpeed; //inverts line up speeds in auto
        RobotContainer.m_LimelightSubsystem.side = "Red"; //switches limelight pose to use red side instead of blue maybe
        //this line may need tweaking will ask brandon if hes free
        
    }
    

}
