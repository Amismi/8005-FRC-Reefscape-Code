package frc.robot.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.LimelightHelpers;

public class Limelight {
    String limelightName;

    public Limelight(String name)
    {
        this.limelightName = name;
    }

    NetworkTable table = LimelightHelpers.getLimelightNTTable(limelightName);

    public double getTX()
    {
        return table.getEntry("tx").getDouble(0);
    }

    public double getTY()
    {
        return table.getEntry("tx").getDouble(0);
    }
    
    public boolean getTV()
    {
        return LimelightHelpers.getTV(limelightName); 
    }

    public double AngleComparedToTag()
    {
        return 0;
        //idk how to get this gotta look at the other thing
    }
    
}
