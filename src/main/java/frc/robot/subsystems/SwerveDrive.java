/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerveUtils.SwerveModule;
import frc.robot.subsystems.swerveUtils.SwerveOdometry;

public class SwerveDrive extends SubsystemBase {

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private ChassisSpeeds speeds;
  private final SwerveOdometry odometry;

  AHRS ahrs;


  public SwerveDrive() {

    frontLeft = new SwerveModule(Constants.Swerve.frontLeftDrive, Constants.Swerve.frontLeftSpin);
    frontRight = new SwerveModule(Constants.Swerve.frontRightDrive, Constants.Swerve.frontRightSpin);
    backLeft = new SwerveModule(Constants.Swerve.backLeftDrive, Constants.Swerve.backLeftSpin);
    backRight = new SwerveModule(Constants.Swerve.backRightDrive, Constants.Swerve.backRightSpin);

    odometry = new SwerveOdometry(1, 1);

    ahrs = new AHRS(SPI.Port.kMXP);

    frontLeft.setSpinPIDConstants(2, 0.002, 0);
    frontLeft.setSpinEncoderInverted(true);
    frontLeft.setDriveMotorInverted(true);

    frontRight.setSpinPIDConstants(1.5, 0.002, 0);
    frontRight.setSpinEncoderInverted(true);
    frontRight.setDriveMotorInverted(true);

    backLeft.setSpinPIDConstants(1.25, 0.001, 0);
    backLeft.setSpinEncoderInverted(true);
    backLeft.setDriveMotorInverted(true);

    backRight.setSpinPIDConstants(1.3, 0.002, 0);
    backRight.setSpinEncoderInverted(true);
    backRight.setDriveMotorInverted(true);

    speeds = new ChassisSpeeds(0, 0, 0);

  }

  /**
   * A simple function that returns the NavX value scoped 
   * @return NavX in range -180 to 180
   */
  public double getCurrentAngle(){
    double ang = ahrs.getYaw();
    if (ang > 180) ang -= 360;
    //flip yaw 
    ang *= -1;

    return ang;
  }

  /**
   * Simple function to print NavX values
   */
  public void debugNavX(){
    SmartDashboard.putNumber("NavX", getCurrentAngle());
  }
  
  /**
   * Function to set the speeds of the drivebase
   * @param speeds (x translation -1 to 1, y translation -1 to 1, rotation -1 to 1)
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    this.speeds = speeds;
  }

  public double getPosAcross(){
    return odometry.getPosAcrossField();
  }

  public double getPosAlong(){
    return odometry.getPosAlongField();
  }

  //I wish the person reading this a very nice day!
  @Override
  public void periodic(){

    SwerveModuleState[] states = new SwerveModuleState[4];

    //Get robot yaw
    double yaw = getCurrentAngle();

    //Get back joystick data
    double rightY = speeds.vxMetersPerSecond;
    double rightX = speeds.vyMetersPerSecond;
    double leftX = speeds.omegaRadiansPerSecond;

    //Get constants
    double frConstant = Constants.Swerve.frSpin;
    double brConstant = Constants.Swerve.brSpin;
    double flConstant = Constants.Swerve.flSpin;
    double blConstant = Constants.Swerve.blSpin;


    double speedTranslation = MathUtil.clamp(Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2)), 0.0, 1.0);
    double directionTranslation = Math.toDegrees(Math.atan2(rightX, rightY));
    double speedRotation = -leftX;


    double frAngle = calculateAngle(speedTranslation, directionTranslation, speedRotation , yaw, frConstant);
    double frPow = calculatePower(speedTranslation, directionTranslation, speedRotation , yaw, frConstant);


    double brAngle = calculateAngle(speedTranslation, directionTranslation, speedRotation , yaw, brConstant);
    double brPow = calculatePower(speedTranslation, directionTranslation, speedRotation , yaw, brConstant);


    double flAngle = calculateAngle(speedTranslation, directionTranslation, speedRotation , yaw, flConstant);
    double flPow = calculatePower(speedTranslation, directionTranslation, speedRotation , yaw, flConstant);


    double blAngle = calculateAngle(speedTranslation, directionTranslation, speedRotation , yaw, blConstant);
    double blPow = calculatePower(speedTranslation, directionTranslation, speedRotation , yaw, blConstant);


    //FL
    states[0] = new SwerveModuleState(flPow, Rotation2d.fromDegrees(flAngle));
    //FR
    states[1] = new SwerveModuleState(frPow, Rotation2d.fromDegrees(frAngle));
    //BL
    states[2] = new SwerveModuleState(blPow, Rotation2d.fromDegrees(blAngle));
    //BR
    states[3] = new SwerveModuleState(brPow, Rotation2d.fromDegrees(brAngle));

    odometry.update(getCurrentAngle(), frPow, flPow, brPow, blPow, frAngle, flAngle, blAngle, brAngle);

    frontLeft.setModuleState(states[0]);
    frontRight.setModuleState(states[1]);
    backLeft.setModuleState(states[2]);
    backRight.setModuleState(states[3]);

    SmartDashboard.putNumber("X", odometry.getPosAcrossField());
    SmartDashboard.putNumber("Y", odometry.getPosAlongField());

  }

    /**
     * Math can be found on the swerve2020 channel in the pins
     * @param s Speed of translation
     * @param d Direction of translation
     * @param r Speed of rotation
     * @param p Rotational position
     * @param c Angle constant per module
     * @return Angle in degrees 
     */
    public double calculateAngle(double s, double d, double r, double p, double c){
      double dpc = d - p - c;
      double atan2P1 = s * Math.sin(Math.toRadians(dpc));
      double atan2P2 = r + s * Math.cos(Math.toRadians(dpc));
      double angle = c + Math.toDegrees(Math.atan2(atan2P1, atan2P2));

      //Wrap angle
      if (angle > 180) angle -= 360;
      if (angle < -180) angle += 360;

      return angle;
    }

    /**
    * Math can be found on the swerve2020 channel in the pins
     * @param s Speed of translation
     * @param d Direction of translation
     * @param r Speed of rotation
     * @param p Rotational position
     * @param c Angle constant per module
     * @return Speed 0-1
     */
    public double calculatePower(double s, double d, double r, double p, double c){

      double dpc = d - p - c;
      double uncorrected = Math.sqrt(Math.pow(r, 2) + Math.pow(s, 2) + (2 * r * s) * Math.cos(Math.toRadians(dpc)));
      double correction = (Math.max(s, Math.abs(r))) / (s + Math.abs(r));

      double ret = uncorrected * correction;

      if (Double.isNaN(ret)) ret = 0;

      return ret;
    }

}
