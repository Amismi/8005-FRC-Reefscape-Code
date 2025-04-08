package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    private double limelightMaxSpeed = 0.007;
    double elevatorLevel = 0;

        public final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        //.withDeadband(MaxSpeed *0.01)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); //open loop control for motors
        //motors don't adjust based on output or self correct in open loop, only adjust based on controller inputs
        //realistically during teleop, it should be open loop otherwise it could be much harder to drive with a lot of feedback
        //but in auto it should be closed looped to allow for robot to correct where it is based on pid controllers given by the 
        //ppholonomic controller class so that it can be a more accurate self adjusting auto

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

    public Command AlignDrivetrain(CommandSwerveDrivetrain drivetrain, int Pipeline, double RightSpeed, double ForwardSpeed)
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
                    .withVelocityX(ForwardSpeed)
                    .withVelocityY(LimelightHelpers.getTX(limelight) * -RightSpeed)
                    .withRotationalRate(limelightSubsystem.CurrentLimelight(topLimelight, bottomLimelight).angLock() * 0.25)
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
        max.stopMotor();
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

        //Extremeties Auto Commands
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
        
        NamedCommands.registerCommand("Left Lineup", AlignDrivetrain(drivetrain, 1, limelightMaxSpeed, elevatorLevel))

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
    }



}
