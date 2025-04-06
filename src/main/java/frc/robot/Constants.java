package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {

        private Constants()
        {
                //no constructor for you :)
        }

        // MotorIds
        public static final int liftMotorIdNum = 11;
        public static final int elevatorMotorNum = 2;
        public static final int pivotIdNum = 5;
        public static final int algae1IdNum = 1;
        public static final int algae2IdNum = 4;
        public static final int coralIdNum = 3;
    
    
        //EncoderPositions
        public static final double climberDown = 0;
        public static final double climberUp = 1.0;
        
        //PID Vals
        public static final double SteerKP = 25;
        public static final double SteerKI = 0;
        public static final double SteerKD = 1;

        public static final double DriveKP = 5;
        public static final double DriveKI = 0;
        public static final double DriveKD = 1;

        public static final double gravCompensation = 0.01;
        public static final double lift0 = 0.0;
        public static final double lift1 = 1;
        public static final double lift2 = 2;
        public static final double lift3 = 3;
        public static final double lift4 = 4;
        public static final double processorLevel = 5;
        public static final double recieveLevel = 6;
    
        public static final double algae1Closed = 0.0;
        public static final double algae1Open = 1.0;
        public static final double algae2Closed = 0.0;
        public static final double algae2Open = 1.0;
        public static final double coralClosed = 0.0;
        public static final double coralOpen = 1.0;
        public static final double pivotUp = 1.0;
        public static final double pivotDown = -0.5;

        //traj constants
        public static final TrapezoidProfile.Constraints kThetaControllerConstaints =
                new TrapezoidProfile.Constraints(Math.PI, Math.PI);

        //limelight names
        
        public final static String limeLightOneName = "limelight-bottom";
        public final static String limelightTwoName = "limelight-top";

        //elevator position
        public final static double elevatorDefaultPos = 1.5;
        public final static double elevatorPos1 = 13.5;
        public final static double elevatorPos2 = 43;
        public final static double elevatorIntakePos = 15.5;
        public final static double elevatorPos3 = 86;
        public final static double elevatorAlgeaPos1 = 31;
        public final static double elevatorAlgeaPos2 = 61;

        //limelight constants
        public static final double limelightMountAngleDegrees = 25;
        public static final double limeilghtLensHeightInches = 9.17;
        public static final double aprilTagHeight = 9;

        

        //robot poses 
        public final static Pose2d testPose = new Pose2d(9, 4, new Rotation2d(0.12));


}
