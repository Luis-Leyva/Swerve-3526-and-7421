// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.Drive;
import frc.robot.subsystems.SwerveChassis;

public class RobotContainer {

	SwerveChassis chassis = new SwerveChassis();
	XboxController controller = new XboxController(0);

	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		NamedCommands.registerCommand("Test Command", Commands.print("Test Command"));

		autoChooser = AutoBuilder.buildAutoChooser();

		configureBindings();
	}

	private void configureBindings() {
		chassis.setDefaultCommand(new Drive(chassis, controller));
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
