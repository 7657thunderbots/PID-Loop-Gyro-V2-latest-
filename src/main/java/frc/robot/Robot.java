package frc.robot;
 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {
    final double bkP = -0.01;
    final double bkI = -0.01;
    final double bkD = -0.001;
    final double biLimit = 3;
    double setpoint = 0;
    double errorSum = 0;
    double lastTimestamp = 0;
    double lastError = 0;
    private double Speedvar=0.0;
    private double turnerror =0.0;
    double berror=0;
    double errorRate=0;
    public final Timer wait = new Timer();


    private static final String kDefaultSpeed = "Demo";
    private static final String kCompetitionSpeed = "Competition";
    private String speed_selected;
    private final SendableChooser<String> speed_chooser = new SendableChooser<>();


    






     double akP = 0.25;
    final double akI = 0.4;
    final double akD = 0.0;
    final double aiLimit = .5;
    public double dsetpoint=0;
    private double derrorSum = 0;
    private double dlastError=0;
    private boolean floppywrist = false;
    double dsensorPosition=0;

    private double doutputSpeed;

    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private boolean placed ;
    private boolean waited;
    private boolean waited2;
    private boolean try_balancing;
    private boolean taxied;
    private boolean place2= false;
    
    int timer;
    
    public double speedMult;

    private final Timer m_timer = new Timer();
    public Joystick left;
    public Joystick right;
    public XboxController controller2;
    private boolean onchargestation= false;
    
    public DriveTrain drivetrain;
    
    private Hand Hand;

    private Wrist wrist;

    private Elbow elbow;

    private Shoulder shoulder;
    
    private Pneumatics pneumatics;

    private color_sensor color_sensor;
   

    @Override
  public void robotInit() {
    
    speed_chooser.setDefaultOption("DemoSpeed", kDefaultSpeed);
    speed_chooser.addOption("Competition Speed", kCompetitionSpeed);
    SmartDashboard.putData("Speed choices", speed_chooser);
    place2 = false;
    placed = false;
    speedMult = .7;
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
     // This creates our drivetrain subsystem that contains all the motors and motor control code
     
     drivetrain = new DriveTrain();
      
     elbow = new Elbow();

     wrist = new Wrist();

      shoulder = new Shoulder();

      Hand = new Hand();
     
     pneumatics = new Pneumatics();

   // balancing = new Balancing();

    //  color_sensor = new color_sensor();

    //  auto1 = new Auto1();
    
    // auto2_balance = new Auto2_balance();
    
    //  auto3 = new Auto3();

    left = new Joystick(0);
		right = new Joystick(1);
		controller2 = new XboxController(2);
   drivetrain.m_gyro.reset();
   pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  

  @Override
  public void robotPeriodic() { 
    shoulder.Shoulder_Run();
    elbow.ElbowRun();  
    Hand.Hand_Run();
    wrist.Wrist_Run();
    drivetrain.run_drive();
    drivetrain.getAverageEncoderDistance();
    pneumatics.Run_Pneumatics();
    SmartDashboard.putNumber("tilt angle",drivetrain.m_gyro.getYComplementaryAngle());
    SmartDashboard.putNumber("foward distance", drivetrain.getAverageEncoderDistance());
    SmartDashboard.putNumber("Turn angle", drivetrain.m_gyro.getAngle());
    SmartDashboard.putNumber("distance", dsensorPosition);
    SmartDashboard.putNumber("output", doutputSpeed);
    SmartDashboard.getNumber("elbow", elbow.Elbowencoder.getPosition());
    SmartDashboard.getNumber("Shoulder", shoulder.shouldere.getPosition());
    dsensorPosition=drivetrain.getAverageEncoderDistance();
  }
  


  
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    drivetrain.m_gyro.reset();
    
    m_timer.reset();
		m_timer.start();

    drivetrain.setbrake(true);

    pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    Hand.hsetpoint=-20;

    waited=false;
    waited2=false;
    onchargestation=false;
    try_balancing=false;
    
    dsetpoint=0;
    akP=.25;

    drivetrain.setbrake(true);
    
  }


  @Override
  public void autonomousPeriodic() {
    
    
    switch (m_autoSelected) {
        case kCustomAuto:
        // put in code here
          break;
        
        
          case kDefaultAuto:
          default:
          SmartDashboard.getBoolean("wait",waited);
          double dt = Timer.getFPGATimestamp() - lastTimestamp;
          
          if (try_balancing==false){
           elbow.Esetpoint=-39.071121;
           shoulder.Ssetpoint = 150.377;
           elbow.EkP=0.05;
           // calculations
           double derror = dsetpoint - dsensorPosition;

          if (Math.abs(derror) < aiLimit) {
            derrorSum += derror * dt;
           }
   
           double derrorRate = (derror - dlastError) / dt;
   
            doutputSpeed = (akP * derror + akI * derrorSum + akD * derrorRate);
           //drivetrain.tankDrive(doutputSpeed,doutputSpeed,false);
          //  drivetrain.mywatchdog();
          //  SmartDashboard.getNumber("output", doutputSpeed);
           // output to motors
          // double Speedvar = doutputSpeed;
           // update last- variables
           lastTimestamp = Timer.getFPGATimestamp();
           dlastError = derror;
           
           if (((Math.abs(dsetpoint-dsensorPosition))<.05)){
             placed=true;
             doutputSpeed =0;
            //  derrorSum = 0;
            //  lastTimestamp = 0;
            //  dlastError = 0;
            }
            // drivetrain.tdrive(doutputSpeed, doutputSpeed,false);
          
          if (placed == true && elbow.Elbowencoder.getPosition()<-30){
             wrist.Wsetpoint=-20;
            placed=false;
            place2=true;
          }

          if (wrist.wriste.getPosition() < -13 && place2==true ){
            waited=true;
            place2=false;
           
          }
          if (waited==true){
            Hand.hsetpoint = 0;
            pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
          }
          if (waited==true && Hand.hande.getPosition()> -3 ){
            wrist.Wsetpoint=0;
            waited=false;
            akP=.15;
            dsetpoint=2;
            waited2=true;
          }
          if (waited2==true && dsensorPosition>.6 && dsensorPosition<.5){
            akP=.25;
          }
          if (waited2==true && dsensorPosition>.3){
            elbow.EkP=0.01;
          elbow.Esetpoint = 0;
          shoulder.Ssetpoint=0;
          }
          if (waited2==true && dsensorPosition > .6){
          dsetpoint = 3.9;
          // elbow.EkP=0.01;
          // elbow.Esetpoint = 0;
          // shoulder.Ssetpoint=0;
          }
        if (waited2==true && dsensorPosition<3.7 && floppywrist==false){
        //  dsetpoint=60;
          taxied=true;
          waited2=false;
          floppywrist=true;
        }
        if (taxied==true && dsensorPosition<3.7){
          onchargestation=true;
          try_balancing=true;

        }   
        
        }
        if (onchargestation==true){
          dsensorPosition=dsetpoint;
      if (drivetrain.m_gyro.getYComplementaryAngle()<3 && drivetrain.m_gyro.getYComplementaryAngle()>-3){
        //chargestationbalance=true;
        Speedvar=0;
        drivetrain.setbrake(false);
      }
    
        else {
              setpoint = 0;
     
            // get sensor position
            Double sensorPosition = drivetrain.m_gyro.getYComplementaryAngle();
            // calculations
            berror = setpoint - sensorPosition;
            
            if (Math.abs(berror) < biLimit) {
              errorSum += berror * dt;
            }
    
            errorRate = (berror - lastError) / dt;
    
            Double outputSpeed = bkP * berror + bkI * errorSum + bkD * errorRate;
    
            // output to motors
            Speedvar=outputSpeed;
    
            // update last- variables
            lastTimestamp = Timer.getFPGATimestamp();
            lastError = berror;
          }
          if (Speedvar>.2){
            Speedvar=.2;
            }
            if (Speedvar<-.2){
              Speedvar=-.2;}
  
           doutputSpeed = Speedvar;
        }
          
            if (drivetrain.m_gyro.getAngle()>3){
              turnerror = .05;
              }
              else if (drivetrain.m_gyro.getAngle()<2.5 && drivetrain.m_gyro.getAngle() >-2.5){
              turnerror =0;
              }
              else if (drivetrain.m_gyro.getAngle()<-3){
                turnerror =-.05;
            }
            
           
    
                drivetrain.tdrive(-turnerror+doutputSpeed,turnerror+doutputSpeed, false);
             

        break;
      
    
    }
  }




@Override
public void teleopInit(){
drivetrain.setbrake(true);
} 

@Override
  public void teleopPeriodic() {
    // DataLogManager.start();
    
    //drivetrain.setbrake(false);
    speed_selected = speed_chooser.getSelected();
     SmartDashboard.putString("Speed Chosen", speed_selected);

      if (controller2.getRightStickButton()){
          setpoint = 0;
     
          if (drivetrain.m_gyro.getYComplementaryAngle()<3 && drivetrain.m_gyro.getYComplementaryAngle()>-3){
            //chargestationbalance=true;
            Speedvar=0;
            drivetrain.setbrake(false);
          }
          else{



            // get sensor position
            Double sensorPosition = drivetrain.m_gyro.getYComplementaryAngle();
    
            // calculations
            berror = setpoint -sensorPosition;
            double dt = Timer.getFPGATimestamp() - lastTimestamp;
    
            if (Math.abs(berror) < biLimit) {
              errorSum += berror * dt;
            }
    
            errorRate = (berror - lastError) / dt;
    
            Double outputSpeed = bkP * berror + bkI * errorSum + bkD * errorRate;
    
            // output to motors
            Speedvar=outputSpeed;
    
            // update last- variables
            lastTimestamp = Timer.getFPGATimestamp();
            lastError = berror;
          }
          
            if (Speedvar>.2){
              Speedvar=.2;
              }
            if (Speedvar<-.2){
                Speedvar=-.2;}
    
               double directionL= Speedvar;
               double directionR= Speedvar;
               drivetrain.tdrive(directionL,directionR, false);
              }
          else if(left.getTrigger()){
            drivetrain.arcadeDrive(left.getY()*speedMult,right.getX()*speedMult, false);
            drivetrain.setbrake(true);
          }
          else {
            drivetrain.setbrake(true);
            drivetrain.tankDrive(right.getY() * speedMult, left.getY() * speedMult,false);
              drivetrain.mywatchdog(); 
          }


   
      //Relese Hand
      if (controller2.getXButton()) {
        Hand.hsetpoint = 0;
      pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      } 
      //Grab Hand 
        else if (controller2.getYButton()) {
          Hand.hsetpoint=-20;
    pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
      }
    //Low place 
    if (controller2.getAButton()) {
      elbow.Esetpoint=-23;
      elbow.EkP=0.05;
      } 
      //Home
     if (controller2.getRightTriggerAxis()>.1) {
      wrist.Wsetpoint=0;
      elbow.EkP=0.01;
      elbow.Esetpoint = 0;
      shoulder.Ssetpoint=0;
    }
    if (controller2.getRightBumper()){
      elbow.Esetpoint=-9;
      elbow.EkP=.09;
     
    }
    //Shelf grab
    if (controller2.getBButton()){
      elbow.Esetpoint=-28; 
      elbow.EkP=0.05;
      //drivetrain.m_gyro.reset();
      
    }
   //Wrist tilt up
    if (controller2.getPOV()==0){
      wrist.Wsetpoint= 5;
    
    }

        if (controller2.getBackButton()){
         // drivetrain.m_gyro.reset();
        }
      
       //high cone
       if (controller2.getLeftTriggerAxis()>.1) {
          elbow.Esetpoint=-39.071121;
          shoulder.Ssetpoint = 150.377;
          elbow.EkP=0.05;
        } 
         
         
       //Wrist zero and place 
       if(controller2.getPOV()==180){
         wrist.Wsetpoint=0;
        }
         else if (controller2.getLeftBumper()){
         wrist.Wsetpoint=-20;  
       }

}


}