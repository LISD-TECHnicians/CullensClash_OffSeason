// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.FuelConstants.DRIVE_DEADBAND;
import static frc.robot.Constants.OperatorConstants.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CANDriveSubsystem;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.DriveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new Drive. */
  CANDriveSubsystem driveSubsystem;
  CommandXboxController controller;
  BooleanSupplier slow;
  private double drive_scale;
  private  double rotate_scale;

  public Drive(CANDriveSubsystem driveSystem, CommandXboxController driverController, BooleanSupplier slowSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    controller = driverController;
    slow = slowSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (slow.getAsBoolean()) {
        drive_scale = DRIVE_SCALING / 2; 
        rotate_scale = ROTATION_SCALING / 2;
    }
    else {
      drive_scale = DRIVE_SCALING;
      rotate_scale = ROTATION_SCALING;
    } 
  }

  // Called every time the scheduler runs while the command is scheduled.
  // The Y axis of the controller is inverted so that pushing the
  // stick away from you (a negative value) drives the robot forwards (a positive
  // value). The X axis is scaled down so the rotation is more easily
  // controllable.
  @Override
  public void execute() {
    driveSubsystem.driveArcade(MathUtil.applyDeadband(controller.getLeftY(), DRIVE_DEADBAND) * drive_scale, 
      MathUtil.applyDeadband(controller.getRightX(), DRIVE_DEADBAND) * rotate_scale);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
