// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain) {
    addCommands(
      new TurnDegrees(1, 20.724, drivetrain),
      //lift claw arm a little bit,
      new DriveDistance(1, 223.622, drivetrain),
      new TurnDegrees(1, 69.276, drivetrain),
      //put claw down,
      new DriveDistance(1, 123.2283, drivetrain),
      //isnert pneumatic claw open commend,
      new DriveDistance(-1, 257.0866, drivetrain),
      new TurnDegrees(1, 10.164, drivetrain),
      new DriveDistance(1, 33.07087, drivetrain),
      //isert claw clouse cod her e,
      new TurnDegrees(1, 44.901, drivetrain),
      new DriveDistance(1, 208.661, drivetrain),
      new TurnDegrees(1, -90, drivetrain),
      new DriveDistance(1, 40.94488, drivetrain),
      new TurnDegrees(1, 90, drivetrain),
      new DriveDistance(1, 11.811, drivetrain),
      //insert claw open cod here,
      new DriveDistance(1, -71.25984, drivetrain),
      new yAxisStabilization(1, false, false, 0, drivetrain)
      // new DriveDistance(1, 123.228, drivetrain)
    );
    // addCommands(

    //   );
        // new DriveDistance(-0.5, 10, drivetrain),
        // new TurnDegrees(-0.5, 180, drivetrain),
        // new DriveDistance(-0.5, 10, drivetrain),
        // new TurnDegrees(0.5, 180, drivetrain));
  }
}
