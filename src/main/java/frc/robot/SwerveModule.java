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

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;


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
        
        //NOTE: currently REVs PID controller doesn't support continous input, when they do, switch it to use the CANPIDcontroller

        //spinPIDController = spinMotor.getPIDController();
        //spinPIDController.setFeedbackDevice(spinAnalogEncoder);  
        spinPIDController = new edu.wpi.first.wpilibj.controller.PIDController(0.02, 0, 0, 0.02);
        spinPIDController.enableContinuousInput(-0.5, 0.5);  
        spinPIDController.setIntegratorRange(-0.5, 0.5);     
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
        //Magic number comes from empirics, should be 13.0666...
        pos *= 1/13.07389;
        pos = (pos % 1 + 1) % 1;

        return pos;/// 360;
    }

    public void debug(){
        SmartDashboard.putNumber("BR revs from motor", spinRevEncoder.getPosition());
        SmartDashboard.putNumber("BR pos from lamprey", spinAnalogEncoder.getPosition());
        SmartDashboard.putNumber("Corrected BR", getScopedEncoderPos());
    }

    /**
     * Set the motors to their desired position using a desired module state (speed and power)
     * @param state Your desired output vector of the robot
     */

    public void setModuleState(SwerveModuleState state) {

        //If the controller input is zero and we're at our setpoint, just stop 
        if(state.speedMetersPerSecond == 0 && spinPIDController.atSetpoint()) {
            driveMotor.set(0);
            spinMotor.set(0);
            return;
        }
          
        double curPos = getScopedEncoderPos();
        double driveSpeed = state.speedMetersPerSecond / Constants.Swerve.maxSpeed;


        //This is where we want to go
        double target = state.angle.getDegrees() / 360;
        double delta = Math.abs(curPos - target);

        if (delta > 0.25 && delta < 0.75){
            driveSpeed *= -1;
            target -= 0.5;
        } 

        double move = MathUtil.clamp(spinPIDController.calculate(curPos, target), -1, 1);
 
        spinMotor.set(move);
        driveMotor.set(driveSpeed);

    }


}

