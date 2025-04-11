// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ToolSubsystems;
import frc.robot.subsystems.Vision.LimelightSubsystem;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;


public class RobotContainer {

    //motor intialization

    public final static TalonFX lift = new TalonFX(Constants.liftMotorIdNum);
    public final static SparkMax algaeright = new SparkMax(Constants.algae1IdNum, MotorType.kBrushless);
    public final static SparkMax algaeleft = new SparkMax(Constants.algae2IdNum, MotorType.kBrushless);
    public final static SparkMax coral = new SparkMax(Constants.coralIdNum, MotorType.kBrushless);
    public final static TalonFX elevatorMotor = new TalonFX(Constants.elevatorMotorNum);
    public final static SparkMax pivot = new SparkMax(Constants.pivotIdNum, MotorType.kBrushless);

    //subsystem intialization
    
    public static ToolSubsystems m_ToolSubsystems = new ToolSubsystems();
    public static LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
    public static Commands CommandSystem = new Commands(); //this is the most helpful system that holds all commands aka 
    //things the robot can do
    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    
    //speed variables
    public static double MaxBaseSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedVar = 1; //variable to control the percentage of max speed the robot goes, currently at max
    private double speedAngleVar = .70; //same thing as last but for rotating speed
    public static double MaxSpeed = MaxBaseSpeed; //variable to switch to negative if on red side and controls are flipped
    private double limelightMaxSpeed = 0.007; //percentage of speed drivetrain goes aligning, low so the robot doesn't oscilate since tx can get high
    public static double baseautoLineUpSpeed = 0.1; //max speed to go when the robot goes forward during aligning in auto
    public static double autoLineUpSpeed = baseautoLineUpSpeed; //used to flip speed to negative if on red side and sides are flipped
    private double joystickForwardSpeed = 0; //variables that 
    private double joystickStrafeSpeed = 0;
    private double joystickAngleSpeed = 0;
    

    //intializing the controllers to use

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);
    
    /* Setting up bindings for necessary control of the swerve drive platform */

    //different objects that dictate the way that the drivetrain can move, field and robot centric 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 15% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    //no deadband on robot centric so that it doesn't screw with limelight align auto
    public final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        //.withDeadband(MaxSpeed *0.01)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); //open loop control for motors aka not using 
        //output data to change the input. PIDS do do this so this isn't pid based.

    //creating drivetrain object
    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    

    //creating the dropdown menu with the autos in dashboard
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {

        //creating the auto commands
        CommandSystem.RegisterNamedCommands(drivetrain);
        //building the pathplanner that the robot ends up using along with named commands
        autoChooser = AutoBuilder.buildAutoChooser(); 
        //pushing the auto commands to the dashboard
        SmartDashboard.putData("AutoMode", autoChooser);
        //doing all the bindings and saying what happens when we press a button
        //this function should run every frame realistically.
        configureBindings();
    }


    private void configureBindings() {
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        //may have to change this if it doesn't work periodically
        joystickForwardSpeed = -joystick.getLeftY() * MaxSpeed * -speedVar;
        joystickStrafeSpeed = -joystick.getLeftX() * MaxSpeed * -speedVar;
        joystickAngleSpeed = -joystick.getRightX() * MaxAngularRate * speedAngleVar;
        //speeds that the drivetrain runs based on joystick and what we want max speed to be
         
        m_ToolSubsystems.SetConfig();
      
        
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            CommandSystem.DriveDrivetrain(
                drivetrain, 
                joystickForwardSpeed, 
                joystickStrafeSpeed, 
                joystickAngleSpeed
            )
        );
    
                
        //limelight align
        joystick.leftTrigger(0.3).whileTrue(
        CommandSystem.AlignDrivetrain(
            drivetrain,
            0, //have to change pipeline var to the limelight var
            limelightMaxSpeed, 
            joystickForwardSpeed, 
            true, 
            true)
        );

        joystick.rightTrigger(0.3).whileTrue(
        CommandSystem.AlignDrivetrain(
            drivetrain, 
            0, 
            limelightMaxSpeed, 
            joystickForwardSpeed, 
            true,
            false)
        );


        // lift
        joystick2.a().whileTrue(CommandSystem.MoveLift(true));
        joystick2.a().or(joystick2.b()).onFalse(CommandSystem.StopLift);
        joystick2.b().whileTrue(CommandSystem.MoveLift(true));

        //pivot
        joystick2.povUp().onTrue(CommandSystem.MovePivot("Intake"));
        joystick2.povDown().onTrue(CommandSystem.MovePivot("Outake"));
        joystick2.povRight().or(joystick2.povLeft()).onTrue(CommandSystem.MovePivot("Default"));


        //algae motors
        joystick2.leftBumper().whileTrue(CommandSystem.MoveAlgea(false));
        joystick2.leftBumper().or(joystick2.rightBumper()).onFalse(CommandSystem.StopAlgea());
        joystick2.rightBumper().whileTrue(CommandSystem.MoveAlgea(false));

        //coral motors ---flopped right & left
        joystick2.rightTrigger().whileTrue(CommandSystem.MoveMotor(coral, true));
        joystick2.leftTrigger().or(joystick2.rightTrigger()).onFalse(CommandSystem.StopMotor(coral));
        joystick2.leftTrigger().whileTrue(CommandSystem.MoveMotor(coral, false));


        //elevator motors
        joystick.povRight().onTrue(CommandSystem.ElevatorLevel("Default"));
        joystick.a().onTrue(CommandSystem.ElevatorLevel("Level 1"));
        joystick.b().onTrue(CommandSystem.ElevatorLevel("Level 2"));
        joystick.y().onTrue(CommandSystem.ElevatorLevel("Level 3"));
        joystick.x().onTrue(CommandSystem.ElevatorLevel("Default"));
        joystick.leftBumper().onTrue(CommandSystem.ElevatorLevel("Algae 1"));
        joystick.rightBumper().onTrue(CommandSystem.ElevatorLevel("Algae 2"));

        //reset elevator encoders
        joystick.povLeft().or(joystick.povRight()).onTrue(new SequentialCommandGroup
            (new InstantCommand(() -> m_ToolSubsystems.MoveElevatorMotor(0)), 
            new WaitCommand(0.75),
            new InstantCommand(() -> m_ToolSubsystems.ResetElevatorEncoder()))
        );

        //resets pivot and elevator
        joystick2.x().onTrue(CommandSystem.ResetRobotParts);
        
        //to be removed because it should be fixed, inverts the controls with a button
        joystick2.y().onTrue(new InstantCommand(() -> m_ToolSubsystems.FlipMaxSpeed()));
        
        
        
        //send all the telemetry to the dashboard
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    //auto command to run, running the one chosen in the dashboard
    public Command getAutonomousCommand() {
       return autoChooser.getSelected();
    }

    
}
