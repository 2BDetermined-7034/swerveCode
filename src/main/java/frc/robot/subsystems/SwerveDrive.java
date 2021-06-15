// 0.047 P Value

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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

    frontLeft = new SwerveModule(Constants.Swerve.frontLeftDrive, Constants.Swerve.frontLeftSpin, new Translation2d(distanceToCenter, distanceToCenter));
    frontRight = new SwerveModule(Constants.Swerve.frontRightDrive, Constants.Swerve.frontRightSpin, new Translation2d(distanceToCenter, distanceToCenter));
    backLeft = new SwerveModule(Constants.Swerve.backLeftDrive, Constants.Swerve.backLeftSpin, new Translation2d(distanceToCenter, distanceToCenter));
    backRight = new SwerveModule(Constants.Swerve.backRightDrive, Constants.Swerve.backRightSpin, new Translation2d(distanceToCenter, distanceToCenter));

    kinematics = new SwerveDriveKinematics(frontLeft.getLocation(), frontRight.getLocation(), backLeft.getLocation(), backRight.getLocation());

    frontLeft.setSpinPIDConstants(3, 0.003, 0);
    frontLeft.setSpinEncoderInverted(true);
    frontRight.setSpinPIDConstants(2.3, 0.002, 0);
    frontRight.setSpinEncoderInverted(true);
    backLeft.setSpinPIDConstants(3, 0.003, 0);
    backLeft.setSpinEncoderInverted(true);
    backRight.setSpinPIDConstants(2.3, 0.003, 0);
    backRight.setSpinEncoderInverted(true);

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

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    this.speeds = speeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    frontLeft.setModuleState(states[0]);
    frontRight.setModuleState(states[1]);
    backLeft.setModuleState(states[2]);
    backRight.setModuleState(states[3]);
    System.out.println("Hi");
    /*
    //Jakob I wrote code that works -Sam
    SmartDashboard.putNumber("Analog number", currentModule.getSpinAnlogEncoder().getVoltage());
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
