// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.nio.channels.Pipe;
import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


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
    public static Commands CommandSystem = new Commands();
    
    
    //speed variables
    public static double MaxBaseSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedVar = 1; //variable to control the percentage of max speed the robot goes, currently at max
    private double speedAngleVar = .70; //same thing as last but for rotating speed
    public static double MaxSpeed = MaxBaseSpeed; //variable to switch to negative if on red side and controls are flipped
    private double limelightMaxSpeed = 0.007; //percentage of speed drivetrain goes aligning, low so the robot doesn't oscilate since tx can get high
    public final static double baseautoLineUpSpeed = 0.1; //max speed to go when the robot goes forward during aligning in auto
    public static double autoLineUpSpeed = baseautoLineUpSpeed; //used to flip speed to negative if on red side and sides are flipped
    
    //object to send values to the dashboard from telemetry file
    private final Telemetry logger = new Telemetry(MaxSpeed);

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
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); //open loop control for motors
        //motors don't adjust based on output or self correct in open loop, only adjust based on controller inputs
        //realistically during teleop, it should be open loop otherwise it could be much harder to drive with a lot of feedback
        //but in auto it should be closed looped to allow for robot to correct where it is based on pid controllers given by the 
        //ppholonomic controller class so that it can be a more accurate self adjusting auto

    //creating drivetrain object
    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    

    //creating the dropdown menu with the autos in dashboard
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {


        // Registering all named commands, used in pathplanner, do the command supplied to them during auto if you so choose
        //NamedCommands.registerCommand("Score Coral Left", new SequentialCommandGroup());
        NamedCommands.registerCommand("Elevator Level One", new InstantCommand(()-> m_ToolSubsystems.MoveElevatorMotor(Constants.elevatorPos1)));
        NamedCommands.registerCommand("Elevator Level Two", new InstantCommand(()-> m_ToolSubsystems.MoveElevatorMotor(Constants.elevatorPos2)));
        NamedCommands.registerCommand("Elevator Level Three", new InstantCommand(()-> m_ToolSubsystems.MoveElevatorMotor(Constants.elevatorPos3)));
        NamedCommands.registerCommand("Elevator Level Default", new InstantCommand(()-> m_ToolSubsystems.MoveElevatorMotor(Constants.elevatorDefaultPos)));

        //NamedCommands.registerCommand("Intake Coral", new InstantCommand(()-> m_ToolSubsystems.IntakeCoral()));
        NamedCommands.registerCommand("Coral Out",new RunCommand(()-> m_ToolSubsystems.MoveMotorForwards(coral)).withTimeout(0.5));        
        NamedCommands.registerCommand("Coral In",new RunCommand(()-> m_ToolSubsystems.MoveMotorBackwards(coral)).withTimeout(1.5));
        NamedCommands.registerCommand("Pivot Default", new InstantCommand(()-> m_ToolSubsystems.PivotDefault()));
        NamedCommands.registerCommand("Pivot Intake", new InstantCommand(()-> m_ToolSubsystems.PivotIntake()));
        NamedCommands.registerCommand("Pivot Outake", new InstantCommand(()-> m_ToolSubsystems.PivotOutTake()));
        NamedCommands.registerCommand("Algae Intake", new InstantCommand(()-> m_ToolSubsystems.MoveAlgea(true)).withTimeout(Time.ofBaseUnits(1, Seconds)));
        NamedCommands.registerCommand("Algae Outake", new InstantCommand(()-> m_ToolSubsystems.MoveAlgea(false)).withTimeout(Time.ofBaseUnits(1, Seconds)));

        NamedCommands.registerCommand("Elevator Algae Bottom", new InstantCommand(()-> m_ToolSubsystems.MoveElevatorMotor(Constants.elevatorAlgeaPos1)));
        NamedCommands.registerCommand("Elevator Algae Top", new InstantCommand(()-> m_ToolSubsystems.MoveElevatorMotor(Constants.elevatorAlgeaPos2)));
        NamedCommands.registerCommand("Stop Coral", new InstantCommand(() -> m_ToolSubsystems.StopMotor(coral)));
        NamedCommands.registerCommand("Reset Encoder", new InstantCommand(() -> m_ToolSubsystems.ResetElevatorEncoder()));

        NamedCommands.registerCommand("Reset Robot Parts", new InstantCommand(() -> m_ToolSubsystems.ResetRobotParts()));
        
        //Named commands that take over drive train for a certain amount of time to allow for aligning with apriltags
        
        NamedCommands.registerCommand("Left Lineup", new SequentialCommandGroup(
            new InstantCommand(() ->  LimelightHelpers.setPipelineIndex(Constants.limeLightOneName, 1)),
            drivetrain.applyRequest(() ->
                robotCentricDrive
                    .withVelocityY(LimelightHelpers.getTX(Constants.limeLightOneName) * -limelightMaxSpeed)
                    .withRotationalRate(m_LimelightSubsystem.angLock() * 0.25)
            )
        )
        .withTimeout(2.5)
        );

        NamedCommands.registerCommand("Right Lineup", new SequentialCommandGroup(
            new InstantCommand(() ->  LimelightHelpers.setPipelineIndex(Constants.limeLightOneName, 0)),
            drivetrain.applyRequest(() ->
                robotCentricDrive
                    .withVelocityY(LimelightHelpers.getTX(Constants.limeLightOneName) * -limelightMaxSpeed)
                    .withRotationalRate(m_LimelightSubsystem.angLock() * 0.25)
            )
        )
        .withTimeout(2.5)
        ); //this one may have to be switched if the pipeline indexes are wrong
        //have to switch index first otherwise the robot may track wrong and make the error way too high
        
        NamedCommands.registerCommand("Go To Coral", drivetrain.applyRequest(() ->
            robotCentricDrive
                .withVelocityX(MaxSpeed * -autoLineUpSpeed)
                .withVelocityY(LimelightHelpers.getTX(Constants.limeLightOneName) * -limelightMaxSpeed)
                .withRotationalRate(m_LimelightSubsystem.angLock() * .25)
            )
            .withTimeout(0.2)
        );

        NamedCommands.registerCommand("Leave Coral", drivetrain.applyRequest(() ->
            robotCentricDrive
                .withVelocityX(MaxSpeed * autoLineUpSpeed)
                .withVelocityY(LimelightHelpers.getTX(Constants.limeLightOneName) * -limelightMaxSpeed)
                .withRotationalRate(m_LimelightSubsystem.angLock() * .25)
            )
            .withTimeout(0.3)
        );
        //named command to stop robot because otherwise the drivetrain just drives with its last request
        NamedCommands.registerCommand("Stop Robot", drivetrain.applyRequest(() ->
            robotCentricDrive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
            )
            .withTimeout(.01)
        );

        NamedCommands.registerCommand("Go To Intake", drivetrain.applyRequest(() ->
        robotCentricDrive
            .withVelocityX(MaxSpeed * -autoLineUpSpeed)
            .withVelocityY(LimelightHelpers.getTX(Constants.limelightTwoName) * -limelightMaxSpeed)
            .withRotationalRate(m_LimelightSubsystem.angLock() * .25) // align rz here
        )
        .withTimeout(1) // may have to change line up speed and timing
    );
        NamedCommands.registerCommand("Leave Intake", drivetrain.applyRequest(() ->
        robotCentricDrive
            .withVelocityX(MaxSpeed * autoLineUpSpeed)
            .withVelocityY(LimelightHelpers.getTX(Constants.limelightTwoName) * -limelightMaxSpeed)
            .withRotationalRate(m_LimelightSubsystem.angLock() * .25) // align rz here
        )
        .withTimeout(0.8) // may have to change line up speed and timing
    );

        //sequential commands to simplify things, use named commands to make even simpler
        //just build in pathplanner then when it works just convert to code
        SequentialCommandGroup ScoreRight = new SequentialCommandGroup(
            NamedCommands.getCommand("Right Lineup"),
            new WaitCommand(.5),
            NamedCommands.getCommand("Elevator Level Three"),
            new WaitCommand(.5),
            NamedCommands.getCommand("Pivot Outake"),
            new WaitCommand(1),
            NamedCommands.getCommand("Go To Coral"),
            new WaitCommand(1),
            NamedCommands.getCommand("Stop Robot"),
            new WaitCommand(.5),
            NamedCommands.getCommand("Coral Out"),
            new WaitCommand(.5),
            NamedCommands.getCommand("Stop Coral"),
            new WaitCommand(.5),
            NamedCommands.getCommand("Leave Coral"),
            NamedCommands.getCommand("Reset Robot Parts")
        );
        SequentialCommandGroup ScoreLeft = new SequentialCommandGroup(
            NamedCommands.getCommand("Left Lineup"),
            new WaitCommand(.5),
            NamedCommands.getCommand("Elevator Level Three"),
            new WaitCommand(.5),
            NamedCommands.getCommand("Pivot Outake"),
            new WaitCommand(1),
            NamedCommands.getCommand("Go To Coral"),
            new WaitCommand(1),
            NamedCommands.getCommand("Stop Robot"),
            new WaitCommand(.5),
            NamedCommands.getCommand("Coral Out"),
            new WaitCommand(.5),
            NamedCommands.getCommand("Stop Coral"),
            new WaitCommand(.5),
            NamedCommands.getCommand("Leave Coral"),
            NamedCommands.getCommand("Reset Robot Parts")
        );

        NamedCommands.registerCommand("Score Right", ScoreRight);
        NamedCommands.registerCommand("Score Left", ScoreLeft);

        //making the dropdown hold all the autos made with pathplanner and putting it on the dashboard 
        //under the name auto modes
        autoChooser = AutoBuilder.buildAutoChooser(); 
        SmartDashboard.putData("AutoMode", autoChooser);


        configureBindings();
    }


    private Command AlignDrivetrain(int Pipeline)
    {
        String limelight;

        if(Pipeline == 2 || Pipeline == 3)
        {
            limelight = Constants.limelightTwoName;
        } else {
            limelight = Constants.limeLightOneName;
        }

        return new SequentialCommandGroup(
            new InstantCommand(() -> LimelightHelpers.setPipelineIndex(limelight, Pipeline)),
            drivetrain.applyRequest(() ->
                robotCentricDrive
                    .withVelocityY(LimelightHelpers.getTX(limelight) * -limelightMaxSpeed)
                    .withRotationalRate(m_LimelightSubsystem.angLock() * 0.25)
                    )
            );
    }


    private void configureBindings() {
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
         
        m_ToolSubsystems.SetConfig();
      
        
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive
                .withVelocityX(-joystick.getLeftY() * MaxSpeed * -speedVar) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed * -speedVar) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate * speedAngleVar)) // Drive counterclockwise with negative X (left)            )
        );
    
                
        //limelight align
        joystick.leftTrigger(0.3).whileTrue(drivetrain.applyRequest(() -> 
        robotCentricDrive.withVelocityY(LimelightHelpers.getTX(Constants.limeLightOneName) * -limelightMaxSpeed)
        .withVelocityX(joystick.getLeftY() * MaxSpeed * speedVar)
        .withRotationalRate(m_LimelightSubsystem.angLock() * .25)
        )
        .alongWith(new InstantCommand(() -> LimelightHelpers.setPipelineIndex(Constants.limeLightOneName, 0))));

        joystick.rightTrigger(0.3).whileTrue(drivetrain.applyRequest(() -> 
        robotCentricDrive.withVelocityY(LimelightHelpers.getTX(Constants.limeLightOneName) * -limelightMaxSpeed)
        .withVelocityX(joystick.getLeftY() * MaxSpeed * speedVar)
        .withRotationalRate(m_LimelightSubsystem.angLock() * .25)
        )
        .
        alongWith(new InstantCommand(() -> LimelightHelpers.setPipelineIndex(Constants.limeLightOneName, 1))));

            

        


        // lift
        joystick2.a().whileTrue(new InstantCommand(()-> m_ToolSubsystems.LiftMotorForward()));
        joystick2.a().or(joystick2.b()).onFalse(new InstantCommand(()-> m_ToolSubsystems.StopLiftMotor()));
        joystick2.b().whileTrue(new InstantCommand(()-> m_ToolSubsystems.LiftMotorBackwards()));

        //pivot
        joystick2.povUp().onTrue(new InstantCommand(()-> m_ToolSubsystems.PivotIntake()));
        //joystick2.povDown().or(joystick2.povUp()).onFalse(new InstantCommand(()-> LiftSubsystem.StopMotor(pivot)));
        joystick2.povDown().onTrue(new InstantCommand(()-> m_ToolSubsystems.PivotOutTake()));
        joystick2.povRight().or(joystick2.povLeft()).onTrue(new InstantCommand(() -> m_ToolSubsystems.PivotDefault()));


        //algae motors
        joystick2.leftBumper().whileTrue(new InstantCommand(()-> m_ToolSubsystems.MoveAlgea(true)));
        joystick2.leftBumper().or(joystick2.rightBumper()).onFalse(new InstantCommand(()-> m_ToolSubsystems.StopAlgae()));
        joystick2.rightBumper().whileTrue(new InstantCommand(()-> m_ToolSubsystems.MoveAlgea(false)));

        //testing align with the intake
        joystick2.povLeft().whileTrue(AlignDrivetrain(m_LimelightSubsystem.CurPipeline(false)));
        joystick2.povLeft().whileTrue(AlignDrivetrain(m_LimelightSubsystem.CurPipeline(true)));


        
        
    

        //coral motors ---flopped right & left
        joystick2.rightTrigger().whileTrue(new InstantCommand(()-> m_ToolSubsystems.MoveMotorForwards(coral)));
        joystick2.leftTrigger().or(joystick2.rightTrigger()).onFalse(new InstantCommand(() -> m_ToolSubsystems.StopMotor(coral)));
        joystick2.leftTrigger().whileTrue(new InstantCommand(()-> m_ToolSubsystems.MoveMotorBackwards(coral)));


        //elevator motors
        joystick.povRight().onTrue(new InstantCommand(()-> m_ToolSubsystems.MoveElevatorMotor(Constants.elevatorDefaultPos)));
        joystick.a().onTrue(new InstantCommand(()-> m_ToolSubsystems.MoveElevatorMotor(Constants.elevatorPos1)));
        joystick.b().onTrue(new InstantCommand(()-> m_ToolSubsystems.MoveElevatorMotor(Constants.elevatorPos2)));
        joystick.y().onTrue(new InstantCommand(() -> m_ToolSubsystems.MoveElevatorMotor(Constants.elevatorPos3)));
         //joystick.povLeft().or(joystick.povRight()).onTrue(new InstantCommand(() -> LiftSubsystem.MoveElevatorMotor(Constants.elevatorIntakePos)));
        joystick.x().onTrue(new InstantCommand(() -> m_ToolSubsystems.MoveElevatorMotor(Constants.elevatorDefaultPos)));
        joystick.leftBumper().onTrue(new InstantCommand(()-> m_ToolSubsystems.MoveElevatorMotor(Constants.elevatorAlgeaPos1)));
        joystick.rightBumper().onTrue(new InstantCommand(()-> m_ToolSubsystems.MoveElevatorMotor(Constants.elevatorAlgeaPos2)));

        //manual elevator motor commands
        joystick.povUp().whileTrue(new InstantCommand(() -> m_ToolSubsystems.moveUpABit()));
        joystick.povUp().or(joystick.povDown()).onFalse(new InstantCommand(() -> m_ToolSubsystems.StopElevator()));
        joystick.povDown().whileTrue(new InstantCommand(() -> m_ToolSubsystems.moveDownABit()));

        //reset elevator encoders
        joystick.povLeft().or(joystick.povRight()).onTrue(new SequentialCommandGroup
            (new InstantCommand(() -> m_ToolSubsystems.MoveElevatorMotor(0)), 
            new WaitCommand(0.75),
            new InstantCommand(() -> m_ToolSubsystems.ResetElevatorEncoder()))
        );

        //resets pivot and elevator
        joystick2.x().onTrue(new InstantCommand(() -> m_ToolSubsystems.ResetRobotParts()));
        
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
