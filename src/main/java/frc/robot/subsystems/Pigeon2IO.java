package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon2IO implements GyroIO {
    private final Pigeon2 pigeon;

    public Pigeon2IO(int id) {
        this.pigeon = new Pigeon2(id);
    }

    @Override
    public double getYaw() {
        return pigeon.getYaw().getValueAsDouble();
    }

    @Override
    public Rotation2d getRotation2d() {
        return pigeon.getRotation2d();
    }

    @Override
    public boolean isConnected() {
        return pigeon.getStickyFaultField().getValue() == 0;
    }

    @Override
    public void reset() {
        pigeon.reset();
    }
}