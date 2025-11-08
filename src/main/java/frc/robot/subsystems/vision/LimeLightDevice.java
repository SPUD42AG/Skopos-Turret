package frc.robot.subsystems.vision;

import frc.robot.Constants.visonConstants.LMConstant;
import frc.robot.utilities.LimelightHelpers;

public class LimeLightDevice {
    private final String name;
    private final int ID;

     LimeLightDevice(LMConstant constant) {
        this.name = constant.name();
        this.ID = constant.ID();
    }

    public double getTX(){
        return LimelightHelpers.getTX(name);
    }
    public double getTY(){
        return LimelightHelpers.getTY(name);
    }
    public double getTA(){
        return LimelightHelpers.getTA(name);
    }
    public boolean hasTarget(){
        return LimelightHelpers.getTV(name);
    }
}
