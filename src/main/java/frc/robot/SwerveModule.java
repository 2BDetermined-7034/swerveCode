/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.SwerveDrive;

/**
 * Add your docs here.
 */
public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax spinMotor;

    private final CANAnalog spinAnalogEncoder;
    private final CANEncoder spinRevEncoder;
    private final edu.wpi.first.wpilibj.controller.PIDController spinPIDController;


    public SwerveModule(int driveMotorId, int spinMotorId) {
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        spinMotor = new CANSparkMax(spinMotorId, MotorType.kBrushless);

        spinRevEncoder = spinMotor.getEncoder();

        spinAnalogEncoder = spinMotor.getAnalog(AnalogMode.kAbsolute);
        

        //spinRevEncoder.setPositionConversionFactor(196/15);
        /*
        spinRevEncoder.setPositionConversionFactor(196/15);
        spinAnalogEncoder.setPositionConversionFactor(1 / 3.3);

        */
        // TODO Velocity Conversion Factor

        //NOTE: currently REVs PID controller doesn't support continous input, when they do, switch it to use the CANPIDcontroller

        //spinPIDController = spinMotor.getPIDController();
        //spinPIDController.setFeedbackDevice(spinAnalogEncoder);  
        spinPIDController = new edu.wpi.first.wpilibj.controller.PIDController(0.02, 0, 0, 0.02);
        spinPIDController.enableContinuousInput(-0.5, 0.5);  
        spinPIDController.setIntegratorRange(-0.005, 0.005);     


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

    /**
     * @return new angle in 0 - 1.0
     */
    public double getScopedEncoderPos(){
        double pos = spinRevEncoder.getPosition();
        pos *= 1350/49;
        pos = Math.floorMod((int) pos, 360);


        //pos = pos % 360;
        //if (pos < 0) pos += 180;
        return pos;
    }
    /**
     * Set the motors to their desired position using a desired module state (speed and power)
     * @param state Your desired output vector of the robot
     */

    public void debug(){
        SmartDashboard.putNumber("aaaaaaa", spinRevEncoder.getPosition());
        SmartDashboard.putNumber("En", spinAnalogEncoder.getPosition());
        SmartDashboard.putNumber("Alt en", getScopedEncoderPos() / 360);
    }


    public void setModuleState(SwerveModuleState state) {

        //If the controller input is zero and we're at our setpoint, just stop 
        if(state.speedMetersPerSecond == 0 && spinPIDController.atSetpoint()) {
            driveMotor.set(0);
            spinMotor.set(0);
            return;
        }
          
        //This is where the wheel is 
        double curPos = getScopedEncoderPos() / 360;
        double driveSpeed = state.speedMetersPerSecond / Constants.Swerve.maxSpeed;


        //This is where we want to go
        double pos = state.angle.getDegrees() / 360;
        SmartDashboard.putNumber("POS", pos);
        double delta = Math.abs(curPos - pos);

        
        if (delta > 0.25 && delta < 0.75){
            driveSpeed *= -1;
            pos -= 0.5;
        } 

        SmartDashboard.putNumber("Cur Pos", curPos);
        SmartDashboard.putNumber("MODDED POS", pos);
        SmartDashboard.putNumber("Delta", delta);
        


        double move = MathUtil.clamp(spinPIDController.calculate(curPos, pos), -1, 1);
 
        
        spinMotor.set(move);
        driveMotor.set(driveSpeed);

    }


}

