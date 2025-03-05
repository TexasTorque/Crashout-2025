package org.texastorque.subsystems;

import org.littletonrobotics.junction.Logger;
import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath.TorquePathingDrivebase;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.swerve.base.TorqueSwerveModule;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.swerve.TorqueSwerveModuleNEO;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Drivebase extends TorqueStatorSubsystem<Drivebase.State> implements Subsystems, TorquePathingDrivebase {

    public static enum State implements TorqueState {
        FIELD_RELATIVE(null),
        ROBOT_RELATIVE(null),
        ALIGN_TO_ANGLE(null),
        PATHING(null);

        public final State parent;

        private State(final State parent) {
            this.parent = parent == null ? this : parent;
        }
    }

    private static volatile Drivebase instance;

    public static final double WIDTH = .53975,
            MAX_VELOCITY = TorqueSwerveModuleNEO.maxVelocity,
            MAX_ANGULAR_VELOCITY = 4 * Math.PI;

    public final Translation2d LOC_FL = new Translation2d(WIDTH / 2, WIDTH / 2),
            LOC_FR = new Translation2d(WIDTH / 2, -WIDTH / 2),
            LOC_BL = new Translation2d(-WIDTH / 2, WIDTH / 2),
            LOC_BR = new Translation2d(-WIDTH / 2, -WIDTH / 2);

    private final TorqueSwerveModule fl, fr, bl, br;

    public TorqueSwerveSpeeds inputSpeeds;
    public final SwerveDriveKinematics kinematics;
    private SwerveModuleState[] swerveStates;

    private final PIDController headingLockPID;
    private final PIDController alignPID;
    private double alignSetpoint = 0;

    private final TorqueNavXGyro gyro = TorqueNavXGyro.getInstance();

    private Drivebase() {
        super(State.FIELD_RELATIVE);

        fl = new TorqueSwerveModuleNEO("Front Left", Ports.FL_MOD);
        fr = new TorqueSwerveModuleNEO("Front Right", Ports.FR_MOD);
        bl = new TorqueSwerveModuleNEO("Back Left", Ports.BL_MOD);
        br = new TorqueSwerveModuleNEO("Back Right", Ports.BR_MOD);

        inputSpeeds = new TorqueSwerveSpeeds(0, 0, 0);
        kinematics = new SwerveDriveKinematics(LOC_FL, LOC_FR, LOC_BL, LOC_BR);
        swerveStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveStates.length; i++)
            swerveStates[i] = new SwerveModuleState();

        headingLockPID = new PIDController(.08, 0, 0);
        headingLockPID.enableContinuousInput(0, 360);
        alignPID = new PIDController(5, 0, 0);
        alignPID.enableContinuousInput(0, 2 * Math.PI);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        setInputSpeeds(new TorqueSwerveSpeeds());

        SmartDashboard.putData("Swerve",
            builder -> {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty(
                    "Front Left Angle", () -> fl.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty(
                    "Front Left Velocity", () -> fl.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty(
                    "Front Right Angle", () -> fr.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty(
                    "Front Right Velocity", () -> fr.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty(
                    "Back Left Angle", () -> bl.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty(
                    "Back Left Velocity", () -> bl.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty(
                    "Back Right Angle", () -> br.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty(
                    "Back Right Velocity", () -> br.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty(
                    "Robot Angle", () -> getHeading().getRadians(), null);
            }
        );
    }

    @Override
    public final void update(final TorqueMode mode) {
        if (wantsState(State.FIELD_RELATIVE) || wantsState(State.ALIGN_TO_ANGLE)) {
            inputSpeeds = inputSpeeds.toFieldRelativeSpeeds(getHeading());
            if (wantsState(State.ALIGN_TO_ANGLE)) {
                SmartDashboard.putBoolean("Is Drivebase Aligned", isAligned());

                if (!isAligned() && !wantsState(State.FIELD_RELATIVE)) {
                    inputSpeeds.omegaRadiansPerSecond = TorqueMath.constrain(alignPID.calculate(getHeading().getRadians(), alignSetpoint), 3);
                } else if (isAligned()) {
                    desiredState = State.FIELD_RELATIVE;
                }
            }
        }

        SmartDashboard.putString("Drivebase State", desiredState.toString());
        swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY);

        if (inputSpeeds.hasZeroVelocity()) {
            manuallySetModuleAngles(
                    swerveStates[0].angle,
                    swerveStates[1].angle,
                    swerveStates[2].angle,
                    swerveStates[3].angle);
        } else {
            fl.setDesiredState(swerveStates[0]);
            fr.setDesiredState(swerveStates[1]);
            bl.setDesiredState(swerveStates[2]);
            br.setDesiredState(swerveStates[3]);
        }

        Logger.recordOutput("Swerve States", swerveStates);
        Logger.recordOutput("Gyro Angle", getHeading());
    }

    @Override
    public void clean(TorqueMode mode) {
        if (mode.isTeleop()) {
            desiredState = desiredState.parent;
        }
    }

    public boolean isAligned() {
        return TorqueMath.toleranced(getHeading().getRadians(), alignSetpoint, .0523); // radians
    }

    public void setInputSpeeds(TorqueSwerveSpeeds inputSpeeds) {
        this.inputSpeeds = inputSpeeds;
    }

    private void manuallySetModuleAngles(final Rotation2d flAngle, final Rotation2d frAngle, final Rotation2d blAngle, final Rotation2d brAngle) {
        fl.setDesiredState(new SwerveModuleState(0, flAngle));
        fr.setDesiredState(new SwerveModuleState(0, frAngle));
        bl.setDesiredState(new SwerveModuleState(0, blAngle));
        br.setDesiredState(new SwerveModuleState(0, brAngle));
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                fl.getPosition(), fr.getPosition(),
                bl.getPosition(), br.getPosition()
        };
    }

    public Rotation2d getHeading() {
        return gyro.getHeadingCCW();
    }

    public void setAlignSetpoint(double setpoint) {
        setState(State.ALIGN_TO_ANGLE);
        alignSetpoint = setpoint;
    }

    public void resetHeading() {
        gyro.setOffsetCW(Rotation2d.fromRadians(0));
        setPose(new Pose2d(0, 0, getHeading()));
    }

    public double getRadius() {
        return WIDTH * Math.sqrt(2);
    }

    @Override
    public double getMaxPathingVelocity() {
        return MAX_VELOCITY;
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(new Translation2d(0, 0), getHeading());
    }

    @Override
    public void setPose(Pose2d pose) {
        gyro.setOffsetCW(pose.getRotation());
    }

    @Override
    public ChassisSpeeds getActualChassisSpeeds() {
        return kinematics.toChassisSpeeds(swerveStates);
    }

    @Override
    public void onBeginPathing() {
        setState(State.PATHING);
    }

    @Override
    public void onEndPathing() {
        setState(State.FIELD_RELATIVE);
    }

    public static synchronized final Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }

    @Override
    public void setCurrentTrajectory(Trajectory trajectory) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCurrentTrajectory'");
    }
}
