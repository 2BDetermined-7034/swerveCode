package frc.robot.subsystems.swerveUtils;

import java.lang.Math;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class SwerveOdometry {

    // https://www.first1684.com/uploads/2/0/1/6/20161347/chimiswerve_whitepaper__2_.pdf
    double forwardSpeed;
    double strafeSpeed;

    double driveLength;
    double driveWidth;

    double timeStep;
    double posAlongField;
    double posAcrossField;

    Timer loopTimer;
    double lastTime;

    public SwerveOdometry(double length, double width){
        driveLength = length;
        driveWidth = width;
        loopTimer = new Timer();
        loopTimer.start();
    }

    /**
     * Function to get the field position
     * @return the forward/backward position (In meters?)
     */
    public double getPosAlongField() {
        return posAlongField;
    }

    /**
     * Function to get the field position
     * @return the
     */
    public double getPosAcrossField() {
        return posAcrossField;
    }

    /**
     * Update the odometry of the drive and integrate into position
     * @param heading the heading of the robot
     * @param wsFR the front right wheel speed
     * @param wsFL the front left wheel speed
     * @param wsBR the back right wheel speed
     * @param wsBL the back left wheel speed
     * @param waFR the front right wheel angle
     * @param waFL the front left wheel angle
     * @param waBL the back left wheel angle
     * @param waBR the back right wheel angle
     */
    public void update(double heading, double wsFR, double wsFL, double wsBR, double wsBL, double waFR, double waFL, double waBL, double waBR){
        double bFL = Math.sin(Math.toRadians(waFL)) * wsFL;
        double bFR = Math.sin(Math.toRadians(waFR)) * wsFR;
        double aBL = Math.sin(Math.toRadians(waBL)) * wsBL;
        double aBR = Math.sin(Math.toRadians(waBR)) * wsBR;

        double dFL = Math.cos(Math.toRadians(waFL)) * wsFL;
        double cFR = Math.cos(Math.toRadians(waFR)) * wsFR;
        double dBL = Math.cos(Math.toRadians(waBL)) * wsBL;
        double cBR = Math.cos(Math.toRadians(waBR)) * wsBR;

        double A = (aBR + aBL) / 2;
        double B = (bFL + bFR) / 2;
        double C = (cFR + cBR) / 2;
        double D = (dFL + dBL) / 2;

        double ROT1 = (B - A) / driveLength;
        double ROT2 = (C - D) / driveWidth;
        double ROT = (ROT1 + ROT2) / 2;

        double FWD1 = ROT * (driveLength / 2) + A;
        double FWD2 = -ROT * (driveLength / 2) + B;
        double FWD = (FWD1 + FWD2) / 2;

        double STR1 = ROT * (driveWidth / 2) + C;
        double STR2 = -ROT * (driveWidth / 2) + D;
        double STR = (STR1 + STR2) / 2;

        double FWDN = FWD * Math.cos(Math.toRadians(heading)) + STR * Math.sin(Math.toRadians(heading)) * Constants.Swerve.maxSpeed;
        double STRN = STR * Math.cos(Math.toRadians(heading)) - FWD * Math.sin(Math.toRadians(heading)) * Constants.Swerve.maxSpeed;

        timeStep = loopTimer.get() - lastTime;

        posAlongField = posAlongField + FWDN * timeStep;
        posAcrossField = posAcrossField + STRN * timeStep;

        lastTime = loopTimer.get();

    }

}
