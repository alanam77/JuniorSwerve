// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveTrain extends SubsystemBase {
  private SwerveModule FL = new SwerveModule(0, 1, 2,0);
  private SwerveModule FR = new SwerveModule(3, 4, 5,0);
  private SwerveModule BL = new SwerveModule(6, 7, 8,0);
  private SwerveModule BR = new SwerveModule(9, 10, 11,0);
  private Pigeon2 IMU = new Pigeon2(12);
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d(Constants.TRACK_WIDTH/2, Constants.WHEELBASE/2), new Translation2d(Constants.TRACK_WIDTH/2, -Constants.WHEELBASE/2), 
  new Translation2d(-Constants.TRACK_WIDTH/2, Constants.WHEELBASE/2), new Translation2d(-Constants.TRACK_WIDTH/2, -Constants.WHEELBASE/2));
  /** Creates a new serveDrive. */
  public SwerveDriveTrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double x, double y, double rot){
    ChassisSpeeds spds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, new Rotation2d(Math.toRadians(IMU.getYaw())));
    SwerveModuleState [] arr = kinematics.toSwerveModuleStates(spds);
    FL.set(arr[0]);
    FR.set(arr[1]);
    BL.set(arr[2]);
    BR.set(arr[3]);

  }
}
