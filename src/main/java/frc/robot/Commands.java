package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.nio.channels.Pipe;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ToolSubsystems;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.Vision.LimelightSubsystem;

public class Commands {
    //create and initialize commands to use in robot container, makes stuff a little cleaner
    Limelight bottomLimelight = new Limelight(Constants.limeLightOneName);
    Limelight topLimelight = new Limelight(Constants.limelightTwoName);
    LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    ToolSubsystems toolSubsystems = new ToolSubsystems();

    double MaxBaseSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    double autoPercentSpeed = .1;
    double autoLineUpSpeed = MaxBaseSpeed * autoPercentSpeed;
    int usingPipeline;

    String limelight;
    double anglock;
    
 

    private double limelightMaxSpeed = 0.007;
    double elevatorLevel = 0;

    public final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    public Command ElevatorLevel(String Level)
    {
        if(Level == "Default")
        {
            elevatorLevel = Constants.elevatorDefaultPos;
        } else if(Level == "Level 1")
        {
            elevatorLevel = Constants.elevatorPos1;
        } else if(Level == "Level 2")
        {
            elevatorLevel = Constants.elevatorPos2;
        } else if(Level == "Level 3")
        {
            elevatorLevel = Constants.elevatorPos3;
        } else if(Level == "Algae 1")
        {
            elevatorLevel = Constants.elevatorAlgeaPos1;
        } else if(Level == "Algae 2")
        {
            elevatorLevel = Constants.elevatorAlgeaPos2;
        }

        return new RunCommand(() -> toolSubsystems.MoveElevatorMotor(elevatorLevel));
    }

    public Command MoveAlgea(boolean forward)
    {
        return new RunCommand(() -> toolSubsystems.MoveAlgea(forward));
    }

    public Command MoveLift(boolean forward)
    {
        return new RunCommand(() -> toolSubsystems.MoveLift());
    }

    public Command StopAlgea()
    {
        return new RunCommand(() -> toolSubsystems.StopAlgae());
    }

    public Command StopLift = new RunCommand(() -> toolSubsystems.StopLift());

    public Command MovePivot(String position)
    {
        if(position == "Intake")
        {
            return new RunCommand(() -> toolSubsystems.PivotIntake());
        } else if(position == "Default")
        {
            return new RunCommand(() -> toolSubsystems.PivotDefault());
        } else if(position == "Outake")
        {
            return new RunCommand(() -> toolSubsystems.PivotOutTake());
        } else{
            return new RunCommand(() -> toolSubsystems.PivotDefault());
        }
    }

    public Command ResetRobotParts = new RunCommand(() -> toolSubsystems.ResetRobotParts());

    public Command AlignDrivetrain(CommandSwerveDrivetrain drivetrain, Integer Pipeline, double RightSpeed, double ForwardSpeed, boolean useAnglock, Boolean LeftAlign)
    {
        if(Pipeline == null)
        {
            usingPipeline = (int)LimelightHelpers.getCurrentPipelineIndex(limelight);
        }
        else if(Pipeline == 2 || Pipeline == 3)
        {
            limelight = Constants.limelightTwoName;
            if(LeftAlign == true)
            {
                usingPipeline = 2; // may need to be adjusted based on what pipeline is which
            } else if(LeftAlign == false)
            {
                usingPipeline = 3;
            }
        }
        else if(Pipeline == 0 || Pipeline == 1) 
        {
            limelight = Constants.limeLightOneName;
            if(LeftAlign == true)
            {
                usingPipeline = 0; // may need to be adjusted based on what pipeline is which
            } else if(LeftAlign == false)
            {
                usingPipeline = 1;
            }
        }

        if(useAnglock)
        {
            if(limelight == Constants.limeLightOneName && useAnglock == true)
            {
                anglock = bottomLimelight.AngleDeadband();
            } else if(limelight == Constants.limelightTwoName && useAnglock == true){
                anglock = topLimelight.AngleDeadband();
            } else
            {
                anglock = 0;
            }
        }
    

        return new SequentialCommandGroup(
            new InstantCommand(() -> LimelightHelpers.setPipelineIndex(limelight, usingPipeline)),
            drivetrain.applyRequest(() ->
                robotCentricDrive
                    .withVelocityX(ForwardSpeed)
                    .withVelocityY(LimelightHelpers.getTX(limelight) * -RightSpeed)
                    .withRotationalRate(anglock * 0.25)
                    )
            );
    }

    public Command MoveMotor(SparkMax max, boolean forward)
    {
        if(forward == true)
        {
            return new RunCommand(() -> toolSubsystems.MoveMotorForwards(max));
        } else {
            return new RunCommand(() -> toolSubsystems.MoveMotorBackwards(max));
        }
    }

    public Command StopMotor(SparkMax max)
    {
        return new RunCommand(() -> max.stopMotor());
    }

    public Command DriveDrivetrain(CommandSwerveDrivetrain drivetrain, double ForwardSpeed, double RightSpeed, double AngularSpeed)
    {
        return new SequentialCommandGroup(
            new InstantCommand(() -> LimelightHelpers.setPipelineIndex(limelight, usingPipeline)),
            drivetrain.applyRequest(() ->
                robotCentricDrive
                    .withVelocityX(ForwardSpeed)
                    .withVelocityY(-RightSpeed)
                    .withRotationalRate(AngularSpeed)
                    )
            );
    }
    
    public void RegisterNamedCommands(CommandSwerveDrivetrain drivetrain)
    {
        //Elevator Auto Commands
        NamedCommands.registerCommand("Elevator Level Default", ElevatorLevel("Default"));
        NamedCommands.registerCommand("Elevator Level One", ElevatorLevel("Level 1"));
        NamedCommands.registerCommand("Elevator Level Two", ElevatorLevel("Level 2"));
        NamedCommands.registerCommand("Elevator Level Three", ElevatorLevel("Level 3"));
        NamedCommands.registerCommand("Elevator Algae Bottom", ElevatorLevel("Algae 1"));
        NamedCommands.registerCommand("Elevator Algae Top", ElevatorLevel("Algae 2"));

        //Extremities Auto Commands
        NamedCommands.registerCommand("Coral Out", MoveMotor(toolSubsystems.coral, true));        
        NamedCommands.registerCommand("Coral In", MoveMotor(toolSubsystems.coral, true));
        NamedCommands.registerCommand("Pivot Default", MovePivot("Default"));
        NamedCommands.registerCommand("Pivot Intake", MovePivot("Intake"));
        NamedCommands.registerCommand("Pivot Outake", MovePivot("Outake"));
        NamedCommands.registerCommand("Algae Intake", MoveAlgea(true));
        NamedCommands.registerCommand("Algae Outake", MoveAlgea(false));

        NamedCommands.registerCommand("Stop Coral", StopMotor(toolSubsystems.coral));

        NamedCommands.registerCommand("Reset Robot Parts", ResetRobotParts);
        
        //Named commands that take over drive train for a certain amount of time to allow for aligning with apriltags
        
        NamedCommands.registerCommand("Left Lineup", AlignDrivetrain(drivetrain, 1, limelightMaxSpeed, 0, true, true)
            .withTimeout(2.5));

        NamedCommands.registerCommand("Right Lineup", AlignDrivetrain(drivetrain, 0, limelightMaxSpeed, 0, true, false)
            .withTimeout(2.5)); //this one may have to be switched if the pipeline indexes are wrong
        //have to switch index first otherwise the robot may track wrong and make the error way too high
        
        NamedCommands.registerCommand("Go To Coral", AlignDrivetrain(drivetrain, null, limelightMaxSpeed, autoLineUpSpeed, true, null)
            .withTimeout(.3));

        NamedCommands.registerCommand("Leave Coral", AlignDrivetrain(drivetrain, null, limelightMaxSpeed, -autoLineUpSpeed, true, null)
        .withTimeout(0.3));
        //named command to stop robot because otherwise the drivetrain just drives with its last request

        NamedCommands.registerCommand("Stop Robot", drivetrain.applyRequest(() ->
            robotCentricDrive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
            )
            .withTimeout(.01) 
        ); // need this to take control of drivetrain after the align auto commands timeouts otherwise they just keep going


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
    }



}
