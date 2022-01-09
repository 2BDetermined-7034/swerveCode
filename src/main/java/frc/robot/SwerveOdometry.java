package frc.robot;

import java.lang.Math;

public class SwerveOdometry {

    double forwardSpeed;
    double strafeSpeed;

    double driveLength;
    double driveWidth;

    public SwerveOdometry(double length, double width){
        driveLength = length;
        driveWidth = width;
    }

    public void update(double wsFR, double wsFL, double wsBR, double wsBL, double waFR, double waFL, double waBL, double waBR){
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


    }

}
