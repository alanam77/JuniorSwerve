// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveTrain;

public class SwerveCommand extends CommandBase {
  XboxController controller;
  SwerveDriveTrain sdrive;
  /** Creates a new serveCommand. */
  public SwerveCommand( XboxController controller, SwerveDriveTrain sdrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.sdrive = sdrive;
    addRequirements(sdrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = -controller.getLeftY();
    double y = -controller.getLeftX();
    double rot = -controller.getRightX();
    sdrive.set(
    x * Constants.MAX_TRANS_METERS_PER_SEC, 
    y * Constants.MAX_TRANS_METERS_PER_SEC, 
    rot * Constants.MAX_ANG_RAD_PER_SEC);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
