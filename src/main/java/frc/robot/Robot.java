package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

import java.text.BreakIterator;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.units.Time;
//import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import java.io.Console;
//import javax.lang.model.util.ElementScanner14;


/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */

public class Robot extends TimedRobot {
 // Compressor pCompressor = new Compressor(0,PneumaticsModuleType.CTREPCM);
  //DoubleSolenoid exampleDoublePCM =new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
  
  Timer timer;

  double time = Timer.getFPGATimestamp();
  double voltage_scale_factor = 1;
  //public DigitalOutput ultrasonicTriggerPinOne = new DigitalOutput(0);
  //public final AnalogInput ultrasonic = new AnalogInput(0);
 // private final AnalogInput ultrasonic = new AnalogInput(0);
  
  
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  //private Joystick m_rightStick;

  private static final int leftDeviceID1 = 1; 
  private static final int leftDeviceID2= 2;
  private static final int rightDeviceID1 = 3; 
  private static final int rightDeviceID2 = 4;

  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax m_rightMotor2;
  
  private static PWMSparkMax atici1 = new PWMSparkMax(0);
  private static PWMSparkMax atici2 = new PWMSparkMax(1);
  private static PWMVictorSPX atici3 = new PWMVictorSPX(2);
  private static PWMVictorSPX atici4 = new PWMVictorSPX(3);
 // public static MotorControllerGroup firlatma = new MotorControllerGroup(atici1,atici2);
  //private static PWMSparkMax asansor = new PWMSparkMax(2);
  //ultrasonic sensor 
   
  /* 
  private static PWMSparkMax leftMotor1 = new PWMSparkMax(0);//cahnge it to 0
  private static PWMSparkMax leftMotor2 = new PWMSparkMax(1);//change it to 1
  private static PWMSparkMax rigthMotor1 = new PWMSparkMax(2);//cahnge it to 0
  private static PWMSparkMax rigthMotor2 = new PWMSparkMax(3);//change it to 1

  public static MotorControllerGroup rightMotorsGroup = new MotorControllerGroup(rigthMotor1,rigthMotor2);
  public static MotorControllerGroup leftMotorsGroup  = new MotorControllerGroup(leftMotor1,leftMotor2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftMotorsGroup,rightMotorsGroup);
  
  */
  private final Joystick m_stick = new Joystick(0);
  
  @Override
  public void robotInit() {

   

    Thread m_visionThread = new Thread(
            () -> {

              
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(640, 480);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(700, 700), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });

    m_visionThread.setDaemon(true);
    m_visionThread.start();
    //ultrasoic sensor
    //SmartDashboard.putNumber("Sensor 1 Range", 500);
    //ultrasonic sensor end
   
    m_leftMotor1 = new CANSparkMax(leftDeviceID1, MotorType.kBrushed);
    m_rightMotor1 = new CANSparkMax(rightDeviceID1, MotorType.kBrushed);
    m_leftMotor2 = new CANSparkMax(leftDeviceID2, MotorType.kBrushed);
    m_rightMotor2 = new CANSparkMax(rightDeviceID2, MotorType.kBrushed);

    
    m_leftMotor1.restoreFactoryDefaults();
    m_rightMotor2.restoreFactoryDefaults();
    m_leftMotor2.restoreFactoryDefaults();
    m_rightMotor1.restoreFactoryDefaults();

 
      m_leftMotor2.follow(m_leftMotor1);
      m_rightMotor2.follow(m_rightMotor1);
    /*m_leftMotor1.follow(m_rightMotor1);
    m_rightMotor2.follow(m_rightMotor1);
    // MotorControllerGroup rightMotorsGroupp = new MotorControllerGroup(m_rightMotor1,m_rightMotor2);
    // MotorControllerGroup leftMotorsGroupp = new MotorControllerGroup(m_leftMotor1,m_leftMotor2);
    */
      m_myRobot = new DifferentialDrive(m_leftMotor1, m_rightMotor1);

      m_leftStick = new Joystick(0);

      //leftMotorsGroupp.setInverted(false);
      //rightMotorsGroupp.setInverted(false);
  /*
      leftMotor1.setSafetyEnabled(true);
      leftMotor1.setExpiration(.1);
      leftMotor1.feed();
      */
      m_myRobot.setSafetyEnabled(true);
      
      
    }

    @Override
    public void autonomousInit() {
    //BİR KERE ÇALIŞIR
      //System.out.println("Auto selected: ");
      timer = new Timer();
      timer.reset();
      timer.start();
      while (timer.get() < 5){

        //atici1.set(-1);
        //atici2.set(-1);
        //atici3.set(-1);
        //atici4.set(-1);

        
        
         //m_myRobot.arcadeDrive(-m_leftStick.getTwist(), -m_leftStick.getY(), true);   

        
      

      
      }
      //timer.reset();
      //timer.start();
      while (timer.get() < 4) {
    //   m_leftMotor1.set(.3);
      //  m_rightMotor1.set(-.3);
      }
      }
    


  /** This function is called periodically during autonomous. */
  // Sürekli Çalışır
  @Override
public void autonomousPeriodic() {
    SmartDashboard.putNumber("time", time);  
    
    // Shooting sequence
    if (time == 2) { 

        atici1.set(-1);
        atici2.set(-1);
        atici3.set(-1);
        atici4.set(-1);
        
        timer.stop();
        timer.reset();
        timer.start();
        
        while (time == 3) {
        atici1.set(0);
        atici2.set(0);
        atici3.set(0);
        atici4.set(0);
        
        }
    }
    
    while (time == 4) {
        atici1.close();
        atici2.close();
        atici3.close();
        atici4.close();
        break;
    }
}

    
  

  
  

  @Override
  public void robotPeriodic() {
    //Publish range readings to SmartDashboard
   // voltage_scale_factor = 5/ RobotController.getVoltage5V();
  }

  @Override
  public void teleopInit() {
  }

  @Override
public void teleopPeriodic() {
   // double rawValue = ultrasonic.getValue();
   // double currentDistanceCentimeters = rawValue * voltage_scale_factor * 0.125;
    //SmartDashboard.putNumber("Sensor 1 Range", currentDistanceCentimeters);

    m_myRobot.arcadeDrive(-m_leftStick.getTwist(), -m_leftStick.getY(), true);

   /*  if (m_leftStick.getRawButton(1)) {
        exampleDoublePCM.set(Value.kForward);
    } else if (m_leftStick.getRawButton(2)) {
        exampleDoublePCM.set(Value.kReverse);
    }*/

    //asansor.set(m_stick.getRawAxis(1));
    

  if (m_leftStick.getRawButton(1)) {
      long start = System.currentTimeMillis();
  
  
      atici1.set(-1);
      atici3.set(-1);

      try {
          Thread.sleep(1000);
      } catch (InterruptedException e) {
          e.printStackTrace(); // Or handle the interruption in an appropriate way
      }

      atici2.set(-1);
      atici4.set(-1);
  }
  

  else if (m_leftStick.getRawButton(2)) {
        atici2.set(0.4);
        atici1.set(0.4);
        atici3.set(0.4);
        atici4.set(0.4);
    }
    else if (m_leftStick.getRawButton(4)) {
      atici2.set(-0.5);
      atici1.set(-0.5);
      atici3.set(-0.5);
      atici4.set(-0.5);
  } 

  else {
        atici1.set(0);
        atici2.set(0);
        atici3.set(0);
        atici4.set(0);
    }
    

  }
}
