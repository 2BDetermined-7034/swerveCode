// 0.047 P Value

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.jni.CANSparkMaxJNI;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveDrive extends SubsystemBase {

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final SwerveDriveKinematics kinematics;

  private ChassisSpeeds speeds;

  AHRS ahrs;

  private final double offsetFL = 0.012; //FL
  private final double offsetFR = 0.001; //FR
  private final double offsetBL = 0.014; //BL
  private final double offsetBR = -0.009; //BR 


  public SwerveDrive() {
    double distanceToCenter = 0.18811;

    frontLeft = new SwerveModule(Constants.Swerve.frontLeftDrive, Constants.Swerve.frontLeftSpin, new Translation2d(distanceToCenter, distanceToCenter), offsetFL);
    frontRight = new SwerveModule(Constants.Swerve.frontRightDrive, Constants.Swerve.frontRightSpin, new Translation2d(distanceToCenter, -distanceToCenter), offsetFR);
    backLeft = new SwerveModule(Constants.Swerve.backLeftDrive, Constants.Swerve.backLeftSpin, new Translation2d(-distanceToCenter, distanceToCenter), offsetBL);
    backRight = new SwerveModule(Constants.Swerve.backRightDrive, Constants.Swerve.backRightSpin, new Translation2d(-distanceToCenter, -distanceToCenter), offsetBR);
            
    ahrs = new AHRS(SPI.Port.kMXP);
  
    kinematics = new SwerveDriveKinematics(frontLeft.getLocation(), frontRight.getLocation(), backLeft.getLocation(), backRight.getLocation());

    frontLeft.setSpinPIDConstants(1.45, 0.002, 0);
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

  
  public double getCurrentAngle(){
    double ang = ahrs.getYaw();
    //if (ang < 0) ang += 360;
    return ang;
  }
  
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    this.speeds = speeds;
  }
  //I wish the person reading this a very nice day!
  @Override
  public void periodic() {
    SmartDashboard.putNumber("YAW", getCurrentAngle());
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    frontLeft.setModuleState(states[0], getCurrentAngle());
    frontRight.setModuleState(states[1], getCurrentAngle());
    backLeft.setModuleState(states[2], getCurrentAngle());
    backRight.setModuleState(states[3], getCurrentAngle());


    //double fl = frontRight.getSpinAnlogEncoder().getVoltage() / 3.3;
    //double fr = frontLeft.getSpinAnlogEncoder().getVoltage() / 3.3;
    //double bl = backLeft.getSpinAnlogEncoder().getVoltage() / 3.3;
    //double br = backRight.getSpinAnlogEncoder().getVoltage() / 3.3;


    //double cfl = frontRight.correct(offsetFR);
    //double cfr = frontRight.correct(offsetFL);
    //double cbl = backLeft.correct(offsetBR);
    //double cbr = backRight.correct(offsetBL);



    /*
    //Jakob I wrote code that works -Sam
    
    SmartDashboard.putNumber("Analog Position", currentModule.getSpinAnlogEncoder().getPosition());
    SmartDashboard.putNumber("Analog Position 2", currentModule.getSpinAnlogEncoder().getPosition());
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    */
  }


}
