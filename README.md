### 6194 Implementation of YAGSL

This implementation of Yet Another Generic Swerve Library is configured for a chassis with:
    - SwerveDriveSpecialties MK4i modules with NEO's and CANCoders
    - NAVx IMU
    - PathPlanner
    - Limelight 2
    - Xbox Controller plugged into port 0
    
### What works!
    - Pathplanner trajectories work beautifully.
    - Driver control works beautifully. 
    - Limelight adds vision measurements IF you've set up you LL for Apriltags and have loaded the Apriltags field layout into LL.
    - Camera position relative to the robot is setup in LL Setup, not in Java

### Controller Bindings
Teleop uses Left stick for translation and right stock for rotation.
Start button zeros the gyro.

### Vendor Dependancy

You need to have the: YAGSL, NavX, PathPlanner, CTRE Phoenix, and REVLib dependencies.

### Known Issues



