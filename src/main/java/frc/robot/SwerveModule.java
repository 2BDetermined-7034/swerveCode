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

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Add your docs here.
 */
public class SwerveModule {

    private final Translation2d location;

    private final CANSparkMax driveMotor;
    private final CANSparkMax spinMotor;

    private final CANAnalog spinAnalogEncoder;
    private final CANPIDController spinPIDController;

    private final double offset;
    
    public SwerveModule(int driveMotorId, int spinMotorId, Translation2d location, double offset_) {
        this.location = location;
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        spinMotor = new CANSparkMax(spinMotorId, MotorType.kBrushless);

        offset = offset_;

        spinAnalogEncoder = spinMotor.getAnalog(AnalogMode.kAbsolute);
        spinAnalogEncoder.setPositionConversionFactor(1 / 3.3);
        

        // TODO Velocity Conversion Factor
        spinPIDController = spinMotor.getPIDController();
        spinPIDController.setFeedbackDevice(spinAnalogEncoder);
        
        

    }

    public Translation2d getLocation() {
        return location;
    }

    public CANAnalog getSpinAnlogEncoder() {
        return spinAnalogEncoder;
    }

    public CANPIDController getSpinPIDController() {
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
<<<<<<< Updated upstream
    public void setModuleState(SwerveModuleState curState, SwerveModuleState state) {
=======
    public void setModuleState(SwerveModuleState curState,SwerveModuleState state) {
>>>>>>> Stashed changes
        if(state.angle.getDegrees() == 0 && state.speedMetersPerSecond == 0) {
            driveMotor.set(0);
            return;
        }
        //if(driveMotor.getDeviceId() != 7) return;
<<<<<<< Updated upstream
        // Desired angle (in degrees)
        double newAngle = state.angle.getDegrees();


        if(newAngle < 0) newAngle += 360;
        //angle = MathUtil.clamp(angle, 30, 330);
        double rot = newAngle / 360; 

        SmartDashboard.putNumber(" rot: ", rot);

        
        
        if(rot < 0.8) {
=======

        /*
            ROTATION CONTROL
                TODO: Fix Rotation jump
        */
        
        SwerveModuleState optimizState = optimize(state, curState.angle.getDegrees());
        
        double angle = optimizState.angle.getDegrees();
        if(angle < 0) angle += 360;
        double rot = angle / 360;


        if(rot < 0.08) {
>>>>>>> Stashed changes
            rot += 0.5;
        }
        if(rot > 0.92) {
            rot -= 0.5;
        }

        spinPIDController.setReference(rot, ControlType.kPosition);

<<<<<<< Updated upstream
        boolean invertDrive = false; 
        invertDrive = rot < 0.9;


        //SmartDashboard.putNumber("SPINNY SPINNY!", state.speedMetersPerSecond);
=======
>>>>>>> Stashed changes

        /*
            SPED CONTROL
                TODO: Should be done with PID and velocity instead
        */

        double driveSpeed = optimizState.speedMetersPerSecond;
        driveMotor.set(driveSpeed);
        //spinPIDController.setReference(0.30, ControlType.kPosition);
    }

<<<<<<< Updated upstream
    public double optimize(double currAngle, double newAngle){

        


        if(newAngle < 0) newAngle += 360;
        //angle = MathUtil.clamp(angle, 30, 330);
        double rot = newAngle / 360; 

        SmartDashboard.putNumber(" rot: ", rot);

        
        
        if(rot < 0.8) {
            rot += 0.5;
        }
        if(rot > 0.92) {
            rot -= 0.5;
        }

        return newAngle;
    }

=======
>>>>>>> Stashed changes
    public double correct(double cons) {
        double start = (spinAnalogEncoder.getVoltage() / 3.3);
        return (start - cons) % 1;
    }


  public SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle, desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle;
    targetSpeed /= 4;
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

