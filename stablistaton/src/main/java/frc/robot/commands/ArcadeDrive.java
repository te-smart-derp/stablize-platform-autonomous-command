// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.pneumatics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final pneumatics m_pneumatics;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;
  private final Supplier<Boolean> m_yaxisWristSupplier;
  private final Supplier<Boolean> m_yaxisClawSupplier;
  private final Supplier<Boolean> m_yaxisTiltSupplier;
  private final Supplier<Boolean> m_WristActivity;
// \
  private final Supplier<Boolean> m_ClawActivity;
  private final Supplier<Boolean> m_TiltActivity;
// image.png

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zaxisRotateSupplier Lambda supplier of rotational speed
   */
  public ArcadeDrive( 
      Drivetrain drivetrain,
      pneumatics pneumatics,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier,
      Supplier<Boolean> yaxisWristSupplier,
      Supplier<Boolean> yaxisClawSupplier,
      Supplier<Boolean> yaxisTiltSupplier,
      Supplier<Boolean> wristActive,
      Supplier<Boolean> clawActive,
      Supplier<Boolean> Tiltactive) {
    m_drivetrain = drivetrain;
    m_pneumatics = pneumatics;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSupplier;
    m_yaxisWristSupplier = yaxisWristSupplier;
    m_yaxisClawSupplier = yaxisClawSupplier;
    m_yaxisTiltSupplier = yaxisTiltSupplier;
    m_ClawActivity = clawActive;
    m_TiltActivity = Tiltactive;
    m_WristActivity = wristActive;
    addRequirements(drivetrain, pneumatics);
  }

//   public ArcadeDrive(Drivetrain m_drivetrain2, Object pneumatics, Supplier<Double> xaxisSpeedSupplier,
// 		Supplier<Double> zaxisRotateSupplier, Supplier<Boolean> yaxisWristSupplier, Supplier<Boolean> yaxisClawSupplier,
// 		Supplier<Boolean> yaxisTiltSupplier, Supplier<Boolean> wristActive, Supplier<Boolean> clawActive,
// 		Supplier<Boolean> tiltactive) {
// }

// public ArcadeDrive(Drivetrain m_drivetrain2, Object pneumatics, Supplier<Double> xaxisSpeedSupplier,
// 		Supplier<Double> zaxisRotateSupplier, Supplier<Boolean> yaxisWristSupplier, Supplier<Boolean> yaxisClawSupplier,
// 		Supplier<Boolean> yaxisTiltSupplier, Supplier<Boolean> wristActive, Supplier<Boolean> clawActive,
// 		Supplier<Boolean> tiltactive) {
// }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_xaxisSpeedSupplier.get(), m_zaxisRotateSupplier.get());
    m_pneumatics.arcadeDrive(m_yaxisWristSupplier.get(), m_yaxisClawSupplier.get(), m_yaxisTiltSupplier.get(), m_ClawActivity.get(), m_TiltActivity.get(), m_WristActivity.get());
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

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.Drivetrain;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import java.util.function.Supplier;

// public class ArcadeDrive extends CommandBase {
//   private final Drivetrain m_drivetrain;
//   private final Supplier<Double> m_xaxisSpeedSupplier;
//   private final Supplier<Double> m_zaxisRotateSupplier;

//   /**
//    * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
//    * lambdas. This command does not terminate.
//    *
//    * @param drivetrain The drivetrain subsystem on which this command will run
//    * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
//    * @param zaxisRotateSupplier Lambda supplier of rotational speed
//    */
//   public ArcadeDrive(
//       Drivetrain drivetrain,
//       Supplier<Double> xaxisSpeedSupplier,
//       Supplier<Double> zaxisRotateSupplier) {
//     m_drivetrain = drivetrain;
//     m_xaxisSpeedSupplier = xaxisSpeedSupplier;
//     m_zaxisRotateSupplier = zaxisRotateSupplier;
//     addRequirements(drivetrain);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_drivetrain.arcadeDrive(m_xaxisSpeedSupplier.get(), m_zaxisRotateSupplier.get());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
