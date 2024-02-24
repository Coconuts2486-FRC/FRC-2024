package frc.robot.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.Odometry;
import frc.robot.Map;
import frc.robot.Misc;
import frc.robot.Swerve;
import frc.robot.Vision.RaspberryPi;

public class AutoMethod {
    public static PIDController straightYPid = new PIDController(1, 0.02, 0);
    public static PIDController rotatePid = new PIDController(.03, 0, 0);

    
    public static void autoDriveY(double distance){
        Swerve.reZeroPosition();
        Map.swerve.drive(0,straightYPid.calculate(Map.odometry.calculatePosition()[1],distance),0);
    }

    public static void autoDriveX(double distance, boolean red){

    }

    public static void driveToGamepiece(){
       RaspberryPi.targetGamePiece(true,false);
    }

    public static void targetApriltag(boolean red){
            
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true, 0, red));    
       
           
        }


    public static void autoRotate(double angle, double target, boolean red){
        if (red == true){
            Map.swerve.drive(0, 0,  rotatePid.calculate(angle, target));
        }
        else{
            Map.swerve.drive(0, 0,  rotatePid.calculate(angle, -target+360));
            
        }
        Map.swerve.realignToField(true);
    }

}
