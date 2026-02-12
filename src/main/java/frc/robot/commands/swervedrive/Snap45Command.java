package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.SwerveDrive;

public class Snap45Command extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final double rotationDegrees; // how much to rotate on button press
    private final double kP = 0.02;       // PID P constant
    private final double maxAngularSpeed; // rad/s

    private Rotation2d targetHeading;

    public Snap45Command(SwerveDrive swerveDrive, double rotationDegrees, double maxAngularSpeed) {
        this.swerveDrive = swerveDrive;
        this.rotationDegrees = rotationDegrees;
        this.maxAngularSpeed = maxAngularSpeed;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        // Compute target heading based on current robot heading
        targetHeading = swerveDrive.getHeading().plus(Rotation2d.fromDegrees(rotationDegrees));
    }

    @Override
    public void execute() {
        // Compute error
        Rotation2d errorHeading = targetHeading.minus(swerveDrive.getHeading());
        double errorDegrees = errorHeading.getDegrees();

        // Wrap to shortest path (-180 to 180)
        if (errorDegrees > 180) errorDegrees -= 360;
        if (errorDegrees < -180) errorDegrees += 360;

        // PID rotation output
        double rotationOutput = kP * errorDegrees;
        rotationOutput = Math.max(-1, Math.min(1, rotationOutput));

        // Drive swerve: zero translation, only rotation
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0, 0,
            rotationOutput * maxAngularSpeed,
            swerveDrive.getHeading()
        );

        SwerveModuleState[] moduleStates = swerveDrive.getKinematics().toSwerveModuleStates(speeds);
        swerveDrive.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop robot
        swerveDrive.drive(new Translation2d(0, 0), 0, true);
    }

    @Override
    public boolean isFinished() {
        // Finish when within 1 degree of target
        double error = targetHeading.minus(swerveDrive.getHeading()).getDegrees();
        return Math.abs(error) < 1.0;
    }
}