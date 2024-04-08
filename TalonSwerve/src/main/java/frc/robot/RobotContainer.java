// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Swerve_Commands;
import frc.robot.Constants.Swerve_Constants;
import frc.robot.Subsystems.Swerve;

public class RobotContainer {

  private final CommandPS4Controller driverController;
  private final CommandPS4Controller operatorController;
  private final Swerve swerveSubsystem;
  private final Swerve_Commands swerve_Commands;


  public RobotContainer() {
    driverController = new CommandPS4Controller(0);
    operatorController = new CommandPS4Controller(1);

    /*Register NamedCommands before creating swerve  */
    swerveSubsystem = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve"));
    swerve_Commands = new Swerve_Commands(swerveSubsystem);
    configureBindings();
  }

  private void configureBindings() {

    Trigger leftBumper = driverController.L1();

    swerveSubsystem.setDefaultCommand(swerve_Commands.drive(
      () -> getAllianceInvert()*MathUtil.applyDeadband(-driverController.getLeftY(), Swerve_Constants.LY_Deadband),
      () -> getAllianceInvert()*MathUtil.applyDeadband(-driverController.getLeftX(), Swerve_Constants.LX_Deadband),
      () -> MathUtil.applyDeadband(-driverController.getRightX(), Swerve_Constants.RX_Deadband),
      () -> !leftBumper.getAsBoolean()
    ));

    driverController.triangle()
    .onTrue(swerve_Commands.zeroGyro());

  }

  public Command getAutonomousCommand() {
    return swerve_Commands.runAuto();
  }

  private double getAllianceInvert(){
    if(DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false){
      return -1;
    }
    return 1;
  }
}
