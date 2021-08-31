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

  private double currentAngle;

  private final double offsetFL = 0.012; //FL
  private final double offsetFR = 0.001; //FR
  private final double offsetBL = 0.014; //BL
  private final double offsetBR = -0.009; //BR 


  //private final Joystick joystick;
  //private final JoystickButton button;

  /*
  private final SwerveModule currentModule;

  double kP = 2;
  double kI = 0.0001;
  double kD = 0;
  */

  public SwerveDrive() {
    double distanceToCenter = 0.18811;

    frontLeft = new SwerveModule(Constants.Swerve.frontLeftDrive, Constants.Swerve.frontLeftSpin, new Translation2d(distanceToCenter, distanceToCenter), offsetFL);
    frontRight = new SwerveModule(Constants.Swerve.frontRightDrive, Constants.Swerve.frontRightSpin, new Translation2d(distanceToCenter, distanceToCenter), offsetFR);
    backLeft = new SwerveModule(Constants.Swerve.backLeftDrive, Constants.Swerve.backLeftSpin, new Translation2d(distanceToCenter, distanceToCenter), offsetBL);
    backRight = new SwerveModule(Constants.Swerve.backRightDrive, Constants.Swerve.backRightSpin, new Translation2d(distanceToCenter, distanceToCenter), offsetBR);

    currentAngle = 0;

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


    // Change this to change what module you're working with for tuning.
    // This will go away when we're done tuning.
    //currentModule = backRight;

    //joystick = new Joystick(0);
    //button = new JoystickButton(joystick, 4);

    /*
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("No Button Goal", 0.3);
    SmartDashboard.putNumber("Button Goal", 0.7);
    currentModule.setSpinEncoderInverted(true);
    currentModule.getSpinPIDController().setP(kP);
    currentModule.getSpinPIDController().setI(kI);
    currentModule.getSpinPIDController().setD(kD);
    */
  }

  public double getCurrentAngle(){
    return currentAngle;
  }
  
  public void setCurrentAngle(double newAngle){
    this.currentAngle = newAngle;
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    this.speeds = speeds;
  }
  //I wish the person reading this a very nice day!
  @Override
  public void periodic() {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    frontLeft.setModuleState(states[0]);
    frontRight.setModuleState(states[1]);
    backLeft.setModuleState(states[2]);
    backRight.setModuleState(states[3]);


    //double fl = frontRight.getSpinAnlogEncoder().getVoltage() / 3.3;
    //double fr = frontLeft.getSpinAnlogEncoder().getVoltage() / 3.3;
    //double bl = backLeft.getSpinAnlogEncoder().getVoltage() / 3.3;
    //double br = backRight.getSpinAnlogEncoder().getVoltage() / 3.3;


    //double cfl = frontRight.correct(offsetFR);
    //double cfr = frontRight.correct(offsetFL);
    //double cbl = backLeft.correct(offsetBR);
    //double cbr = backRight.correct(offsetBL);

    //SmartDashboard.putNumber("Analog number FL", fl);
    //SmartDashboard.putNumber("Analog number FR", fr);
    //SmartDashboard.putNumber("Analog number BL", bl);
    //SmartDashboard.putNumber("Analog number BR", br);

    //SmartDashboard.putNumber("Analog number CFL", cfl);
    //SmartDashboard.putNumber("Analog number CFR", cfr);
    //SmartDashboard.putNumber("Analog number CBL", cbl);
    //SmartDashboard.putNumber("Analog number CBR", cbr);


    /*
    //Jakob I wrote code that works -Sam
    
    SmartDashboard.putNumber("Analog Position", currentModule.getSpinAnlogEncoder().getPosition());
    SmartDashboard.putNumber("Analog Position 2", currentModule.getSpinAnlogEncoder().getPosition());
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);

    if(p != kP) {
      currentModule.getSpinPIDController().setP(p);
      kP = p;
    }

    if(i != kI) {
      currentModule.getSpinPIDController().setI(i);
      kI = i;
    }

    if(d != kD) {
      currentModule.getSpinPIDController().setD(d);
      kD = d;
    }

    double lowerGoal = SmartDashboard.getNumber("No Button Goal", 0);
    double upperGoal = SmartDashboard.getNumber("Button Goal", 0);

    currentModule.getSpinPIDController().setReference(button.get() ? upperGoal : lowerGoal, ControlType.kPosition);
    */
  }


}
