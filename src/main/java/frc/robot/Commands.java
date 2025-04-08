package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    // public Command AlignDrivetrain(CommandSwerveDrivetrain drivetrain, int Pipeline)
    // {
    //     String limelight;

    //     if(Pipeline == 2 || Pipeline == 3)
    //     {
    //         limelight = Constants.limelightTwoName;
    //     } else {
    //         limelight = Constants.limeLightOneName;
    //     }
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> LimelightHelpers.setPipelineIndex(limelight, Pipeline)),
    //         drivetrain.applyRequest(() ->
    //             robotCentricDrive
    //                 .withVelocityY(LimelightHelpers.getTX(limelight) * -limelightMaxSpeed)
    //                 .withRotationalRate(limelightSubsystem.CurrentLimelight(topLimelight, bottomLimelight).angLock() * 0.25)
    //                 )
    //         );
    // }



}
