package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private CANSparkMax shoulder;
  private CANSparkMax intake;
  private CANSparkMax wrist;
  public static Arm currentInstance;

  private SparkMaxPIDController shoulder_pidController;
  private RelativeEncoder shoulder_encoder;
  public double shoulder_kP, shoulder_kI, shoulder_kD, shoulder_kIz, shoulder_kFF, shoulder_kMaxOutput, shoulder_kMinOutput, shoulder_maxRPM, shoulder_maxVel, shoulder_minVel, shoulder_maxAcc, shoulder_allowedErr;
  private SparkMaxPIDController wrist_pidController;
  private RelativeEncoder wrist_encoder;
  public double wrist_kP, wrist_kI, wrist_kD, wrist_kIz, wrist_kFF, wrist_kMaxOutput, wrist_kMinOutput, wrist_maxRPM, wrist_maxVel, wrist_minVel, wrist_maxAcc, wrist_allowedErr;

  public Arm() {
    shoulder = new CANSparkMax(ArmConstants.shoulderMotorID, MotorType.kBrushless);
    intake = new CANSparkMax(ArmConstants.intakeMotorID, MotorType.kBrushless);
    wrist = new CANSparkMax(ArmConstants.wristMotorID, MotorType.kBrushless);

        // initialze PID controller and encoder objects
        shoulder_pidController = shoulder.getPIDController();
        shoulder_encoder = shoulder.getEncoder();
    
        // PID coefficients
        shoulder_kP = 5e-5; 
        shoulder_kI = 0;
        shoulder_kD = 0; 
        shoulder_kIz = 0; 
        shoulder_kFF = 0.000156; 
        shoulder_kMaxOutput = 1; 
        shoulder_kMinOutput = -1;
        shoulder_maxRPM = 11000;
    
        // Smart Motion Coefficients
        shoulder_maxVel = 11000; // rpm
        shoulder_maxAcc = 3000;
    
        // set PID coefficients
        shoulder_pidController.setP(shoulder_kP);
        shoulder_pidController.setI(shoulder_kI);
        shoulder_pidController.setD(shoulder_kD);
        shoulder_pidController.setIZone(shoulder_kIz);
        shoulder_pidController.setFF(shoulder_kFF);
        shoulder_pidController.setOutputRange(shoulder_kMinOutput, shoulder_kMaxOutput);

    int shoulder_smartMotionSlot = 0;
    shoulder_pidController.setSmartMotionMaxVelocity(shoulder_maxVel, shoulder_smartMotionSlot);
    shoulder_pidController.setSmartMotionMinOutputVelocity(shoulder_minVel, shoulder_smartMotionSlot);
    shoulder_pidController.setSmartMotionMaxAccel(shoulder_maxAcc, shoulder_smartMotionSlot);
    shoulder_pidController.setSmartMotionAllowedClosedLoopError(shoulder_allowedErr, shoulder_smartMotionSlot);
    shoulder_encoder.setPositionConversionFactor(1/2.577777);
    shoulder.setSoftLimit(SoftLimitDirection.kForward, 125);
    shoulder.setSoftLimit(SoftLimitDirection.kReverse, -10);
    shoulder.enableSoftLimit(SoftLimitDirection.kForward, true);
    shoulder.enableSoftLimit(SoftLimitDirection.kReverse, true);
    

    // initialze PID controller and encoder objects
    wrist_pidController = wrist.getPIDController();
    wrist_encoder = wrist.getEncoder();

    // PID coefficients
    wrist_kP = 5e-5; 
    wrist_kI = 0;
    wrist_kD = 0; 
    wrist_kIz = 0; 
    wrist_kFF = 0.000156; 
    wrist_kMaxOutput = .5; 
    wrist_kMinOutput = -.5;
    wrist_maxRPM = 5000;

    // Smart Motion Coefficients
    wrist_maxVel = 2000; // rpm
    wrist_maxAcc = 1500;

    // set PID coefficients
    wrist_pidController.setP(wrist_kP);
    wrist_pidController.setI(wrist_kI);
    wrist_pidController.setD(wrist_kD);
    wrist_pidController.setIZone(wrist_kIz);
    wrist_pidController.setFF(wrist_kFF);
    wrist_pidController.setOutputRange(wrist_kMinOutput, wrist_kMaxOutput);

int wrist_smartMotionSlot = 0;
wrist_pidController.setSmartMotionMaxVelocity(wrist_maxVel, wrist_smartMotionSlot);
wrist_pidController.setSmartMotionMinOutputVelocity(wrist_minVel, wrist_smartMotionSlot);
wrist_pidController.setSmartMotionMaxAccel(wrist_maxAcc, wrist_smartMotionSlot);
wrist_pidController.setSmartMotionAllowedClosedLoopError(wrist_allowedErr, wrist_smartMotionSlot);
wrist_encoder.setPositionConversionFactor(1/.225);
    // wrist.setSoftLimit(SoftLimitDirection.kForward, ####);
    // wrist.setSoftLimit(SoftLimitDirection.kReverse, ####);
    // wrist.enableSoftLimit(SoftLimitDirection.kForward, true);
    // wrist.enableSoftLimit(SoftLimitDirection.kReverse, true);


    //Comment this line out if you're done tuning the shoulder
    ShoulderTunerSetup();

    //COmment this line out if you're done tuning the wrist
    wristTunerSetup();

    CoastMode();
    shoulder.setInverted(true);
    wrist.setInverted(true);
    }

  public void setSpeed(double speed) {
    shoulder.set(speed);
    wrist.set(0);
  }


  public void setIntakeSpeed(double speed) {
    intake.set(speed);
  }



/**
   * set the reference position for the should and wrist simulataniously
   *
   * @param shoulderPosition - the position of the shoulder in degrees with 0 being stowed flate
   * @param wristPosition - the position of the shoulder in degrees with 0 being stowed vertically
   */
public void setArmPosition(double shoulderPosition, double wristPosition){
  wrist_pidController.setReference(wristPosition, CANSparkMax.ControlType.kSmartMotion);
  shoulder_pidController.setReference(shoulderPosition, CANSparkMax.ControlType.kSmartMotion);
}


  private void CoastMode() {
    shoulder.setIdleMode(IdleMode.kBrake);
    intake.setIdleMode(IdleMode.kBrake);
    wrist.setIdleMode(IdleMode.kBrake);
  }


private void ShoulderTunerSetup(){
  
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Shoulder/P Gain", shoulder_kP);
    SmartDashboard.putNumber("Shoulder/I Gain", shoulder_kI);
    SmartDashboard.putNumber("Shoulder/D Gain", shoulder_kD);
    SmartDashboard.putNumber("Shoulder/I Zone", shoulder_kIz);
    SmartDashboard.putNumber("Shoulder/Feed Forward", shoulder_kFF);
    SmartDashboard.putNumber("Shoulder/Max Output", shoulder_kMaxOutput);
    SmartDashboard.putNumber("Shoulder/Min Output", shoulder_kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Shoulder/Max Velocity", shoulder_maxVel);
    SmartDashboard.putNumber("Shoulder/Min Velocity", shoulder_minVel);
    SmartDashboard.putNumber("Shoulder/Max Acceleration", shoulder_maxAcc);
    SmartDashboard.putNumber("Shoulder/Allowed Closed Loop Error", shoulder_allowedErr);
    SmartDashboard.putNumber("Shoulder/Set Position", 0);
    SmartDashboard.putNumber("Shoulder/Set Position", 0);
}

private void ShoulderTuner(){
   // read PID coefficients from SmartDashboard
   double p = SmartDashboard.getNumber("Shoulder/P Gain", 0);
   double i = SmartDashboard.getNumber("Shoulder/I Gain", 0);
   double d = SmartDashboard.getNumber("Shoulder/D Gain", 0);
   double iz = SmartDashboard.getNumber("Shoulder/I Zone", 0);
   double ff = SmartDashboard.getNumber("Shoulder/Feed Forward", 0);
   double max = SmartDashboard.getNumber("Shoulder/Max Output", 0);
   double min = SmartDashboard.getNumber("Shoulder/Min Output", 0);
   double maxV = SmartDashboard.getNumber("Shoulder/Max Velocity", 0);
   double minV = SmartDashboard.getNumber("Shoulder/Min Velocity", 0);
   double maxA = SmartDashboard.getNumber("Shoulder/Max Acceleration", 0);
   double allE = SmartDashboard.getNumber("Shoulder/Allowed Closed Loop Error", 0);

   // if PID coefficients on SmartDashboard have changed, write new values to controller
   if((p != shoulder_kP)) { shoulder_pidController.setP(p); shoulder_kP = p; }
   if((i != shoulder_kI)) { shoulder_pidController.setI(i); shoulder_kI = i; }
   if((d != shoulder_kD)) { shoulder_pidController.setD(d); shoulder_kD = d; }
   if((iz != shoulder_kIz)) { shoulder_pidController.setIZone(iz); shoulder_kIz = iz; }
   if((ff != shoulder_kFF)) { shoulder_pidController.setFF(ff); shoulder_kFF = ff; }
   if((max != shoulder_kMaxOutput) || (min != shoulder_kMinOutput)) { 
     shoulder_pidController.setOutputRange(min, max); 
     shoulder_kMinOutput = min; shoulder_kMaxOutput = max; 
   }
   if((maxV != shoulder_maxVel)) { shoulder_pidController.setSmartMotionMaxVelocity(maxV,0); shoulder_maxVel = maxV; }
   if((minV != shoulder_minVel)) { shoulder_pidController.setSmartMotionMinOutputVelocity(minV,0); shoulder_minVel = minV; }
   if((maxA != shoulder_maxAcc)) { shoulder_pidController.setSmartMotionMaxAccel(maxA,0); shoulder_maxAcc = maxA; }
   if((allE != shoulder_allowedErr)) { shoulder_pidController.setSmartMotionAllowedClosedLoopError(allE,0); shoulder_allowedErr = allE; }

   double shoulder_setPoint, shoulder_processVariable;

   shoulder_setPoint = SmartDashboard.getNumber("Shoulder/Set Position", 0);
     /**
      * As with other PID modes, Smart Motion is set by calling the
      * setReference method on an existing pid object and setting
      * the control type to kSmartMotion
      */
    //  shoulder_pidController.setReference(shoulder_setPoint, CANSparkMax.ControlType.kSmartMotion);
     shoulder_processVariable = shoulder_encoder.getPosition();
   
   SmartDashboard.putNumber("Shoulder/SetPoint", shoulder_setPoint);
   SmartDashboard.putNumber("Shoulder/Process Variable", shoulder_processVariable);
   SmartDashboard.putNumber("Shoulder/Output", shoulder.getAppliedOutput());  
}

private void coDriverControlsSetup(){
  SmartDashboard.putBoolean("CoDriver/Cone Mode", true);
  SmartDashboard.putBoolean("CoDriver/Slomo Mode", false);
  SmartDashboard.putNumber("CoDriver/Shoulder Pickup Angle", 47);
  SmartDashboard.putNumber("CoDriver/Wrist Pickup Angle", 75);

}

private void coDriverControls(){
  SmartDashboard.getBoolean("CoDriver/Cone Mode", true);
  SmartDashboard.getBoolean("CoDriver/Slomo Mode", false);
  SmartDashboard.getNumber("CoDriver/Shoulder Pickup Angle", 47);
  SmartDashboard.getNumber("CoDriver/Wrist Pickup Angle", 75);



}


private void wristTunerSetup(){
  
  // display PID coefficients on SmartDashboard
  SmartDashboard.putNumber("Wrist/P Gain", wrist_kP);
  SmartDashboard.putNumber("Wrist/I Gain", wrist_kI);
  SmartDashboard.putNumber("Wrist/D Gain", wrist_kD);
  SmartDashboard.putNumber("Wrist/I Zone", wrist_kIz);
  SmartDashboard.putNumber("Wrist/Feed Forward", wrist_kFF);
  SmartDashboard.putNumber("Wrist/Max Output", wrist_kMaxOutput);
  SmartDashboard.putNumber("Wrist/Min Output", wrist_kMinOutput);

  // display Smart Motion coefficients
  SmartDashboard.putNumber("Wrist/Max Velocity", wrist_maxVel);
  SmartDashboard.putNumber("Wrist/Min Velocity", wrist_minVel);
  SmartDashboard.putNumber("Wrist/Max Acceleration", wrist_maxAcc);
  SmartDashboard.putNumber("Wrist/Allowed Closed Loop Error", wrist_allowedErr);
  SmartDashboard.putNumber("Wrist/Set Position", 0);
  SmartDashboard.putNumber("Wrist/Set Velocity", 0);

}

private void wristTuner(){
 // read PID coefficients from SmartDashboard
 double p = SmartDashboard.getNumber("Wrist/P Gain", 0);
 double i = SmartDashboard.getNumber("Wrist/I Gain", 0);
 double d = SmartDashboard.getNumber("Wrist/D Gain", 0);
 double iz = SmartDashboard.getNumber("Wrist/I Zone", 0);
 double ff = SmartDashboard.getNumber("Wrist/Feed Forward", 0);
 double max = SmartDashboard.getNumber("Wrist/Max Output", 0);
 double min = SmartDashboard.getNumber("Wrist/Min Output", 0);
 double maxV = SmartDashboard.getNumber("Wrist/Max Velocity", 0);
 double minV = SmartDashboard.getNumber("Wrist/Min Velocity", 0);
 double maxA = SmartDashboard.getNumber("Wrist/Max Acceleration", 0);
 double allE = SmartDashboard.getNumber("Wrist/Allowed Closed Loop Error", 0);

 // if PID coefficients on SmartDashboard have changed, write new values to controller
 if((p != wrist_kP)) { wrist_pidController.setP(p); wrist_kP = p; }
 if((i != wrist_kI)) { wrist_pidController.setI(i); wrist_kI = i; }
 if((d != wrist_kD)) { wrist_pidController.setD(d); wrist_kD = d; }
 if((iz != wrist_kIz)) { wrist_pidController.setIZone(iz); wrist_kIz = iz; }
 if((ff != wrist_kFF)) { wrist_pidController.setFF(ff); wrist_kFF = ff; }
 if((max != wrist_kMaxOutput) || (min != wrist_kMinOutput)) { 
   wrist_pidController.setOutputRange(min, max); 
   wrist_kMinOutput = min; wrist_kMaxOutput = max; 
 }
 if((maxV != wrist_maxVel)) { wrist_pidController.setSmartMotionMaxVelocity(maxV,0); wrist_maxVel = maxV; }
 if((minV != wrist_minVel)) { wrist_pidController.setSmartMotionMinOutputVelocity(minV,0); wrist_minVel = minV; }
 if((maxA != wrist_maxAcc)) { wrist_pidController.setSmartMotionMaxAccel(maxA,0); wrist_maxAcc = maxA; }
 if((allE != wrist_allowedErr)) { wrist_pidController.setSmartMotionAllowedClosedLoopError(allE,0); wrist_allowedErr = allE; }

 double wrist_setPoint, wrist_processVariable;

 wrist_setPoint = SmartDashboard.getNumber("Wrist/Set Position", 0);
   /**
    * As with other PID modes, Smart Motion is set by calling the
    * setReference method on an existing pid object and setting
    * the control type to kSmartMotion
    */
  //  wrist_pidController.setReference(wrist_setPoint, CANSparkMax.ControlType.kSmartMotion);
   wrist_processVariable = wrist_encoder.getPosition();
 
 SmartDashboard.putNumber("Wrist/SetPoint", wrist_setPoint);
 SmartDashboard.putNumber("Wrist/Process Variable", wrist_processVariable);
 SmartDashboard.putNumber("Wrist/Output", wrist.getAppliedOutput());  
}


    /**
     * Initialize the current DriveBase instance
     */
    public static void init() {
      if (currentInstance == null) {
          currentInstance = new Arm();
      }
  }

  public static Arm getInstance() {
    init();
    return currentInstance;
}

@Override
public void periodic() {
  //Comment this line out if you're not using the shoulder tuner
     ShoulderTuner();
     wristTuner();
    
    
}
  
}
