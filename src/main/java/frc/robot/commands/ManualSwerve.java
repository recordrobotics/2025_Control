// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.control.IControlInput;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.NavSensor;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class ManualSwerve extends Command {
  public static class ControlOptions {
    private boolean xInverted;
    private boolean yInverted;
    private boolean spinInverted;

    public ControlOptions(boolean xinverted, boolean yinverted, boolean spininverted) {
      xInverted = xinverted;
      yInverted = yinverted;
      spinInverted = spininverted;
    }

    public boolean getXInverted() {
      return xInverted;
    }

    public boolean getYInverted() {
      return yInverted;
    }

    public boolean getSpinInverted() {
      return spinInverted;
    }

    /**
     * Returns a new default instance of the class if obj is null, otherwise returns
     * obj
     * 
     * @param obj Instance or null
     * @return Instance or default instance if obj is null
     */
    public static ControlOptions initNull(ControlOptions obj) {
      if (obj == null)
        return new ControlOptions(false, false, false);
      else
        return obj;
    }
  }

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private Swerve _swerve;
  private IControlInput _controls;

  private SendableChooser<ControlOptions> controlOptions = new SendableChooser<ControlOptions>();

  private Field2d m_field = new Field2d();

  public ChassisSpeeds target;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualSwerve(Swerve swerve, NavSensor nav, IControlInput controls) {
    _swerve = swerve;
    _controls = controls;
    addRequirements(swerve);

    controlOptions.setDefaultOption("N:X, I:SY", new ControlOptions(false, true, true));
    controlOptions.addOption("N:XYS, I:none", new ControlOptions(false, false, false));
    controlOptions.addOption("N:YS, I:X", new ControlOptions(true, false, false));
    controlOptions.addOption("N:XS, I:Y", new ControlOptions(false, true, false));
    controlOptions.addOption("N:S, I:XY", new ControlOptions(true, true, false));
    controlOptions.addOption("N:XY, I:S", new ControlOptions(false, false, true));
    controlOptions.addOption("N:X, I:SY", new ControlOptions(false, true, true));
    controlOptions.addOption("N:Y, I:SX", new ControlOptions(true, false, true));
    controlOptions.addOption("N:none, I:XYS", new ControlOptions(true, true, true));

    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData(controlOptions);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * Target Velocity and Angle
     */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
