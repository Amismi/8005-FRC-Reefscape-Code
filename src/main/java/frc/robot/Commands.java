package frc.robot;

import frc.robot.subsystems.ToolSubsystems;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.Vision.LimelightSubsystem;

public class Commands {
    //create and initialize commands to use in robot container 
    Limelight bottomLimelight = new Limelight(Constants.limeLightOneName);
    Limelight topLimelight = new Limelight(Constants.limelightTwoName);
    LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    ToolSubsystems toolSubsystems = new ToolSubsystems();

    


}
