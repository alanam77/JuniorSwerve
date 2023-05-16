// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ExtraMath;

public class SwerveModule extends SubsystemBase {
  TalonFX rotation;
  TalonFX translation;
  CANCoder rotCoder;
  SwerveModuleState wState;
  PIDController PIDTrans;
  PIDController PIDRotation;
  double offset;
  /** Creates a new serve. */
  public SwerveModule(int tID, int rID, int eID, double offset) {
    rotation = new TalonFX(rID);
    translation = new TalonFX(tID);
    rotCoder = new CANCoder(eID);
    this.offset = offset;
    rotation.configNeutralDeadband(0.001);
    translation.configNeutralDeadband(0.001);

    rotation.setNeutralMode(NeutralMode.Brake);
    translation.setNeutralMode(NeutralMode.Brake);
    PIDTrans = new PIDController(0.00, 0, 0);
    PIDTrans.enableContinuousInput(-Constants.MAX_TRANS_METERS_PER_SEC, Constants.MAX_TRANS_METERS_PER_SEC);
    PIDRotation = new PIDController(0.007, 0, 0.00000001);
    PIDRotation.enableContinuousInput(-180, 180);
    rotCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    translation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rotation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void set(SwerveModuleState wantedState){
    wState = SwerveModuleState.optimize(wantedState, new Rotation2d(Math.toRadians(rotCoder.getAbsolutePosition())));
    double output = ExtraMath.clip(PIDTrans.calculate(translation.getSelectedSensorVelocity(), wState.speedMetersPerSecond), 0.2);
    double feedforward = (wState.speedMetersPerSecond/Constants.MAX_TRANS_METERS_PER_SEC);
    translation.set(ControlMode.PercentOutput, output + feedforward);

    double wantedAngle = ExtraMath.mod(wState.angle.getDegrees() + offset + 180 , 360) - 180;
    double rotoutput = PIDRotation.calculate(rotCoder.getAbsolutePosition(),wantedAngle);
    rotation.set(ControlMode.PercentOutput, rotoutput);
    

  }
}
