package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class NavXIO implements GyroIO {
    private final AHRS navx = new AHRS(SPI.Port.kMXP);

    @Override
    public double getYaw() {
        return navx.getYaw();
    }

    @Override
    public Rotation2d getRotation2d() {
        return navx.getRotation2d();
    }

    @Override
    public boolean isConnected() {
        return navx.isConnected();
    }

    @Override
    public void reset() {
        navx.reset();
    }
}