package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class Odometry {

    // swerve modules
    private Module backRight;
    private Module backLeft;
    private Module frontRight;
    private Module frontLeft;
    private Swerve swerve;

    double aBR = 0.0;
    double aBL = 0.0;
    double bFR = 0.0;
    double bFL = 0.0;
    double cFR = 0.0;
    double cBR = 0.0;
    double dFL = 0.0;
    double dBL = 0.0;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double d = 0.0;

    double z = 0.0;
    double z1 = 0.0;
    double z2 = 0.0;

    double x = 0.0;
    double x1 = 0.0;
    double x2 = 0.0;

    double y = 0.0;
    double y1 = 0.0;
    double y2 = 0.0;

    double x0 = 0.0;
    double y0 = 0.0;

    double W = 30;
    double L = 30;

    double timeDelta = 0.0;
    double lastTime = 0.0;
    double yPos = 0.0;
    double xPos = 0.0;

    public Odometry(Module backRight, Module backLeft, Module frontRight, Module frontLeft, Swerve swerve){

        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.swerve = swerve;
    }

    public void init (){
        timeDelta = 0.0;
        lastTime = 0.0;
        yPos = 0.0;
        xPos = 0.0;
    }
    public double[] calculatePosition(){

        aBR = Math.sin(backRight.getModuleAngle()) * backRight.getModuleSpeed();
        aBL = Math.sin(backLeft.getModuleAngle()) * backLeft.getModuleSpeed();
        bFR = Math.sin(frontRight.getModuleAngle()) * frontRight.getModuleSpeed();
        bFL = Math.sin(frontLeft.getModuleAngle()) * frontLeft.getModuleSpeed();
        cFR = Math.cos(frontRight.getModuleAngle()) * frontRight.getModuleSpeed();
        cBR = Math.cos(backRight.getModuleAngle()) * backRight.getModuleSpeed();
        dFL = Math.cos(frontLeft.getModuleAngle()) * frontLeft.getModuleSpeed();
        dBL = Math.cos(backLeft.getModuleAngle()) * backLeft.getModuleSpeed();

        a = (aBR + aBL) / 2;
        b = (bFR + bFL) / 2;
        c = (cFR + cBR) / 2;
        d = (dFL + dBL) / 2;

        z1 = (b - a) / L;
        z2 = (c - d) / W;
        z = (z1 + z2) / 2;

        y1 = (z * (L / 2)) + a;
        y2 = -(z * (L / 2)) + b;
        y = (y1 + y2) / 2;

        x1 = (z * (W / 2)) + c;
        x2 = -(z * (W / 2)) + d;
        x = (x1 + x2) / 2;

        x0 = x * Math.cos(swerve.getRobotAngle()) - y * Math.sin(swerve.getRobotAngle());
        y0 = y * Math.cos(swerve.getRobotAngle()) + x * Math.sin(swerve.getRobotAngle());

        timeDelta = (150 - Timer.getMatchTime()) - lastTime;
        xPos = xPos + (x0 * timeDelta);
        yPos = yPos + (y0 * timeDelta);

        double[] positions = {xPos, yPos};

        return positions;

    }
    
}
