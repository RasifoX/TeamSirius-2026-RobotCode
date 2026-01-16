package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    double getYaw();

    Rotation2d getRotation2d();

    boolean isConnected();

    void reset();
}