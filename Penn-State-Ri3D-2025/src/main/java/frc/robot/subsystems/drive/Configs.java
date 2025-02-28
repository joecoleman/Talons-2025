package frc.robot.subsystems.drive;


  
public class Configs {

public static class MAXSwerveModule {

    public static DrivingConfig drivingConfig = new DrivingConfig();

    public static TurningConfig turningConfig = new TurningConfig();

}
    
    
    
        public static class DrivingConfig {
    
            public int currentLimit;
    
            public String idleMode; // Add the idleMode field here
    
        }
    
    
    
        public static class TurningConfig {
    
            public int currentLimit;
    
            public String idleMode;
    
        }
    
    }
    


// Other configurations
