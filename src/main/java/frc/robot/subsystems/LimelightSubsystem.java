package frc.robot.subsystems;



import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;




public class LimelightSubsystem implements Subsystem {
    
    public double lastTag;
    public double trueLastTag;
    public boolean isChanging = false;

    //i don't remember why this is here but maybe i'll need it 
    public boolean startMegatag = true;

    public String side = "Blue";


    //getting kinematics for the robot
    public Kinematics robotKinematics = RobotContainer.drivetrain.getKinematics();

    //variable to Estimate the pose, taking in swerve module positions, gyro, poses and many other things
    public final SwerveDrivePoseEstimator estimatePose = new SwerveDrivePoseEstimator((SwerveDriveKinematics) robotKinematics, 
        RobotContainer.drivetrain.getPigeon2().getRotation2d(), 
        new SwerveModulePosition[] {
            RobotContainer.drivetrain.getModule(0).getPosition(true),
            RobotContainer.drivetrain.getModule(1).getPosition(true),
            RobotContainer.drivetrain.getModule(2).getPosition(true),
            RobotContainer.drivetrain.getModule(3).getPosition(true)
            //TO:DO MAKE SURE SWERVE MODULE ORDERS GO FROM FL FR BL BR
        }, 
        new Pose2d(), 
        VecBuilder.fill(0.05, .05, Units.degreesToRadians(5)), 
        VecBuilder.fill(.5, 0.5, Units.degreesToRadians(30)));
    
        //this is code that returns the limelight thats currently seeing a tag, 
        //usually returning the first one, but the second one if one doens't have one and two does
        private String curLimelight()
        {
            if(LimelightHelpers.getTV(Constants.limeLightOneName) == true)
            {
                return Constants.limeLightOneName;
            } else if(LimelightHelpers.getTV(Constants.limelightTwoName) == true){
                return Constants.limelightTwoName;
            } else 
            {
                return Constants.limeLightOneName;
            }
        }
    


    @SuppressWarnings("unchecked")

    //this should be called all the time, and should also be tuned for the modules 
    public void updatePose()
    {
        estimatePose.update(
            RobotContainer.drivetrain.getPigeon2().getRotation2d(),
            new SwerveModulePosition[] {
                RobotContainer.drivetrain.getModule(0).getPosition(true),
                RobotContainer.drivetrain.getModule(1).getPosition(true),
                RobotContainer.drivetrain.getModule(2).getPosition(true),
                RobotContainer.drivetrain.getModule(3).getPosition(true)
                //TO:DO MAKE SURE SWERVE MODULE ORDERS GO FROM FL FR BL BR
            });

        //here in case we need to use mega tag 1 instead, but mega tag 2 is a thousand times more accurate
        boolean useMegaTag2 = true;

        //bool to tell whether to actually update the limelight pose or if its not safe to do so
        boolean doRejectUpdate = false;
        //this entire if statement gets the pose estimate, changing it if it sees the limelight to the megatag,
        // and estimating based on robot kinematics and the latest limelight megatag pose 
        if(useMegaTag2 == true && startMegatag == true) // if we are using mega tag do this 
        {
            //gives robot orientation to library and how fast its moving in each rotation for calculations
            LimelightHelpers.SetRobotOrientation(Constants.limeLightOneName, estimatePose.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

            //variable that gives the pose estimate
            LimelightHelpers.PoseEstimate mt2 = LimelightSide(null);
            //LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(curLimelight()); 
            //will have to change back if this doesn't work            if(Math.abs(RobotContainer.drivetrain.getPigeon2().getRate()) > 720) //if robot is spinning too fast don't use limelight posing, may screw up robot more
            {
                doRejectUpdate = true;
            }
            if(mt2 != null){
            if(mt2.tagCount == 0) // if it cant see any tags, dont update limelight pose
            {
                doRejectUpdate = true;
            }
        }
            if(!doRejectUpdate) // if the other stuff isn't true, update the pose estimate
            {
                estimatePose.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
                estimatePose.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }

    public void getLastTag()
    {
        
        if(LimelightHelpers.getTV(Constants.limeLightOneName))
        {
            isChanging = true;
            lastTag = LimelightHelpers.getFiducialID(Constants.limeLightOneName);
        } else
        {
            isChanging = false;
            lastTag = lastTag; 
        }
    }

    public void getTrueLastTag()
    {
        if(LimelightHelpers.getTV(Constants.limeLightOneName))
        {
            isChanging = true;
            trueLastTag = LimelightHelpers.getFiducialID(curLimelight());
        } else
        {
            isChanging = false;
            trueLastTag = trueLastTag; 
        }
    }
    

    public Rotation2d GetIdRotation()
    {
        //big ass switch statement to get correct rotation
        switch ((int)lastTag) {
            case 7:
                return new Rotation2d(Math.toRadians(0));

            case 6:
                return new Rotation2d(Math.toRadians(-60));

            case 11:
                return new Rotation2d(Math.toRadians(-120));

            case 10:
                return new Rotation2d(Math.toRadians(180));

            case 8:
                return new Rotation2d(Math.toRadians(60));

            case 9:
                return new Rotation2d(Math.toRadians(120));

            case 18:
                return new Rotation2d(Math.toRadians(0));

            case 19:
                return new Rotation2d(Math.toRadians(-60));

            case 20: 
                return new Rotation2d(Math.toRadians(-120));

            case 21:
                return new Rotation2d(Math.toRadians(180));

             case 22:
                return new Rotation2d(Math.toRadians(120));

            case 17:
                return new Rotation2d(Math.toRadians(60));

            default:
                return new Rotation2d(Math.toRadians(0));

        }

    }

    PoseEstimate LimelightSide(String side)
    {
        if(side == "Red")
        {
            return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(curLimelight());
        } else
        {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(curLimelight());
        }
    }


    int[] reefTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
     int[] intakeTags = {1, 2, 12, 13};
    public boolean LookingAtReef;
    public void IsLookingAtReef()
    {
        for(int i = 0; i <= reefTags.length; i++)
        {
            if(trueLastTag == reefTags[i])
            {
                LookingAtReef = true;
            }
        } 
        
        for(int i = 0; i<= intakeTags.length; i++)
        {
            if(trueLastTag == intakeTags[i])
            {
                LookingAtReef = false;
            }
        }        
    }

    //changes pipelines / offsets based on where the limelight is
    public int CurPipeline(boolean rightSide)
    {
        if(LookingAtReef == true)
        {
            if(rightSide == true)
            {
                return 2; 
            } else
            {
                return 3;
            }
        } else
        {
            if(rightSide == true)
            {
                return 0;
            } else
            {
                return 1;
            }
        } 
    } //have to get the right pipelines

             
    public double angLock() {
        double targetAngle = RobotContainer.m_LimelightSubsystem.GetIdRotation().getDegrees();
        double currentAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
        double targetDeg = targetAngle;
        double currentDeg = currentAngle;
        double offset;
        offset = CalculateOffset(targetDeg, currentDeg);
        double angLock = (offset * 3.14/180 * 10);
        // if (angLock < 0){
        //     angLock = angLock * -1;
        // }

        if (offset < 2 && offset > -2) {
            angLock = 0;
        }
        SmartDashboard.putNumber("offset", offset);
        return angLock;
    }

    public double CalculateOffset(double targetAngle, double curAngle)
    {

        double offset = targetAngle - curAngle;
        if(offset > 360)
        {
            offset -=360;
        }else if(offset < -360)
        {
            offset += 360;
        }

        if(offset > 180)
        {
            offset = offset - 360;
        } else if(offset < -180)
        {
            offset = 360 + offset;
        }
        return offset;
    }


}
