/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Add your docs here.
 */
public class SwerveModule {

    private final Translation2d location;

    private final CANSparkMax driveMotor;
    private final CANSparkMax spinMotor;

    private final CANAnalog spinAnalogEncoder;
    private final edu.wpi.first.wpilibj.controller.PIDController spinPIDController;


    public SwerveModule(int driveMotorId, int spinMotorId, Translation2d location, double offset_) {
        this.location = location;
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        spinMotor = new CANSparkMax(spinMotorId, MotorType.kBrushless);


        spinAnalogEncoder = spinMotor.getAnalog(AnalogMode.kAbsolute);
        spinAnalogEncoder.setPositionConversionFactor(1 / 3.3);
        

        // TODO Velocity Conversion Factor
        //spinPIDController = spinMotor.getPIDController();
        spinPIDController = new edu.wpi.first.wpilibj.controller.PIDController(0.03, 0, 0, 0.02);
        //spinPIDController.setFeedbackDevice(spinAnalogEncoder);  
        spinPIDController.enableContinuousInput(-0.5, 0.5);                  

    }

    public Translation2d getLocation() {
        return location;
    }

    public CANAnalog getSpinAnlogEncoder() {
        return spinAnalogEncoder;
    }

    public edu.wpi.first.wpilibj.controller.PIDController getSpinPIDController() {
        return spinPIDController;
    }

    public CANSparkMax getDriveMotor() {
        return driveMotor;
    }

    public CANSparkMax getSpinMotor() {
        return spinMotor;
    }

    public void setSpinEncoderInverted(boolean inverted) {
        spinAnalogEncoder.setInverted(inverted);
    }

    public void setDriveMotorInverted(boolean inverted) {
        driveMotor.setInverted(inverted);
    }

    public void setSpinPIDConstants(double kP, double kI, double kD) {
        spinPIDController.setP(kP);
        spinPIDController.setI(kI);
        spinPIDController.setD(kD);
    }

    // DON'T USE THIS YET. IT ISN'T TESTED.
    // WE ALSO NEED TO ADD THE ACTUAL DRIVE WHEEL MOTORS
    public void setModuleState(SwerveModuleState curState,SwerveModuleState state) {
        if(state.angle.getDegrees() <= 0 && state.speedMetersPerSecond == 0 && spinPIDController.atSetpoint()) {
            driveMotor.set(0);
            spinMotor.set(0);
            return;
        }
   
        double curAngle = curState.angle.getDegrees();
        //if(curAngle < 0) curAngle += 360;
        double curROT = curAngle / 360;
        SwerveModuleState optimizState = optimize(state, curAngle);
        double angle = optimizState.angle.getDegrees();
        
        //if(angle < 0) angle += 360;
        double ROT = angle / 360;
        
        //if (rot == 0) rot = 1;

        SmartDashboard.putNumber("cur Angle", curROT);
        SmartDashboard.putNumber("Want ", ROT);
        spinPIDController.setSetpoint(ROT);

        double move = spinPIDController.calculate(curROT, ROT);
        if (move >= 0.915) move = 0;
        move *=2.5;

        SmartDashboard.putNumber("move", move);
        spinMotor.set(move);


        /*
            SPED CONTROL
                TODO: Should be done with PID and velocity instead
        
        */
        double driveSpeed = optimizState.speedMetersPerSecond;
        driveMotor.set(driveSpeed);

        //spinPIDController.setReference(0.30, ControlType.kPosition);

    }

    public double correct(double cons) {
        double start = (spinAnalogEncoder.getVoltage() / 3.3);
        return (start - cons) % 1;
    }


  public SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle, desiredState.angle.getDegrees());
    
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle;


    targetSpeed /= Constants.Swerve.maxSpeed;
    if (Math.abs(delta) > 90){
        targetSpeed *= -1;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }        
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }


    private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;

      double lowerOffset = scopeReference % 360;
      if (lowerOffset >= 0) {
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + (360 - lowerOffset);
      } else {
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound) {
          newAngle += 360;
      }
      while (newAngle > upperBound) {
          newAngle -= 360;
      }
      if (newAngle - scopeReference > 180) {
          newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
          newAngle += 360;
      }
      return newAngle;
  }


}

