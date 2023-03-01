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
<<<<<<< HEAD
    final double bkP = -0.01;
    final double bkI = -0.01;
    final double bkD = -0.001;
=======
  final double bkP = -0.008;
     final double bkI = -0.005;
     final double bkD = -0.001;
>>>>>>> f3875e76b41260b89cb7ceed590dfb9164ac2482
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

<<<<<<< HEAD

    private static final String kDefaultSpeed = "Demo";
    private static final String kCompetitionSpeed = "Competition";
    private String speed_selected;
    private final SendableChooser<String> speed_chooser = new SendableChooser<>();


    





=======
    final double tkP = -0.008;
     final double tkI = -0.005;
     final double tkD = -0.001;
    final double tiLimit = 3;
    double tsetpoint = 0;
    double terrorSum = 0;
    double tlastError = 0;
    double terror=0;
    double terrorRate=0;
>>>>>>> f3875e76b41260b89cb7ceed590dfb9164ac2482

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

<<<<<<< HEAD
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
    
=======
  private static final String kauto3 = "Auto 3";
  private static final String kauto2 = "Auto 2";
  private static final String kauto1 = "Auto 1";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private boolean start;
  private boolean placed ;
  private boolean waited;
  private boolean waited2;
  private boolean try_balancing;
  private boolean taxied;
  private boolean place2= false;
  private boolean floppywrist = false;
  private boolean turn1ready = false;
  private boolean pickupready = false;
  private boolean pickedup = false;
  private boolean goback = false;
  private boolean turnback = false;
  private boolean preparestart = false;

>>>>>>> f3875e76b41260b89cb7ceed590dfb9164ac2482
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
    
<<<<<<< HEAD
    private Pneumatics pneumatics;

    private color_sensor color_sensor;
   
=======
   private Pneumatics pneumatics;

   private color_sensor color_sensor;
>>>>>>> f3875e76b41260b89cb7ceed590dfb9164ac2482

    @Override
  public void robotInit() {
    place2 = false;
    placed = false;
    speedMult = .7;
    m_chooser.setDefaultOption("auto 2", kauto2);
    m_chooser.addOption("auto1",kauto1);
    m_chooser.addOption("auto 3", kauto3);
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
<<<<<<< HEAD
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
=======
    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
>>>>>>> f3875e76b41260b89cb7ceed590dfb9164ac2482
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
<<<<<<< HEAD
    
    dsetpoint=0;
    akP=.25;

    drivetrain.setbrake(true);
=======
    turn1ready=false;
    dsetpoint=0;
    start=true;
>>>>>>> f3875e76b41260b89cb7ceed590dfb9164ac2482
    
  }


  @Override
  public void autonomousPeriodic() {
<<<<<<< HEAD
    
=======
    double dt = Timer.getFPGATimestamp() - lastTimestamp;
>>>>>>> f3875e76b41260b89cb7ceed590dfb9164ac2482
    
    switch (m_autoSelected) {
        case kauto1:
        // put in code here
          break;
        
        
          case kauto2:
          default:
          SmartDashboard.getBoolean("wait",waited);
          
          
          if (try_balancing==false){
<<<<<<< HEAD
           elbow.Esetpoint=-39.071121;
           shoulder.Ssetpoint = 150.377;
           elbow.EkP=0.05;
=======
            // Hand.hsetpoint=-20;
           // pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
           if(start==true) {
            elbow.Esetpoint=-39.071121;
            shoulder.Ssetpoint = 150.377;
            elbow.EkP=0.05;
            start=false;
            dsetpoint=0;
          }
>>>>>>> f3875e76b41260b89cb7ceed590dfb9164ac2482
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
     
     
     
        case kauto3:
      
           if (start==true){
            elbow.Esetpoint=-39.071121;
            shoulder.Ssetpoint = 150.377;
            elbow.EkP=0.05;
            start=false;
            dsetpoint=0;
           }
           // calculations
           double derror = dsetpoint - dsensorPosition;
           if (Math.abs(derror) < aiLimit) {
            derrorSum += derror * dt;
           }
   
           double derrorRate = (derror - dlastError) / dt;
   
            doutputSpeed = (akP * derror + akI * derrorSum + akD * derrorRate);
          
           // update last- variables
           lastTimestamp = Timer.getFPGATimestamp();
           dlastError = derror;
           
           if (((Math.abs(dsetpoint-dsensorPosition))<.05)){
             placed=true;
             doutputSpeed =0;
            
            }
            
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
          if (waited2==true && dsensorPosition>.4 && dsensorPosition<.5){
            akP=.3;
          }
          if (waited2==true && dsensorPosition > .6){
          dsetpoint = 2;
          elbow.EkP=0.01;
          elbow.Esetpoint = 0;
          shoulder.Ssetpoint=0;
          }
        if (waited2==true && Math.abs(dsensorPosition-dsetpoint)<.1 && floppywrist==false){
          taxied=true;
          waited2=false;
          floppywrist=true;
        }
        if (taxied==true &&Math.abs(dsensorPosition-dsetpoint)<.1){
         turn1ready=true;
        }
        if (turn1ready==true){
          Double tsensorPosition = drivetrain.m_gyro.getAngle();
    
            // calculations
           double terror = tsetpoint-tsensorPosition;
            if (Math.abs(terror) < tiLimit) {
           terrorSum += terror * dt;
            }
    
            double terrorRate = (terror - tlastError) / dt;
    
            Double turn = tkP * terror + tkI * terrorSum + tkD * terrorRate;
    
    
            // update last- variables
            lastTimestamp = Timer.getFPGATimestamp();
            tlastError = terror;
            drivetrain.tdrive(-turn+doutputSpeed,turn+doutputSpeed,false);
            if (turn1ready==true){
              tsetpoint=90;
              turn1ready=false;
              
            }
            if (tsensorPosition<95 && tsensorPosition>85){
              dsetpoint=3;
              pickedup=true;
              
            }
             if (pickedup==true & dsensorPosition <3.1 && dsensorPosition >2.9){
              pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
              Hand.hsetpoint=-20;
              pickedup=false;
              pickupready=true;
             }
             if(pickupready==true && Hand.hande.getPosition()< -5 ){
              wrist.Wsetpoint=5;
              pickupready=false;
              goback = true;
             }
             if (goback==true){
              dsetpoint=2;
              goback=false;
              turnback = true;
             }
             if (turnback==true && dsensorPosition<2.01 && dsensorPosition >1.99){
              tsetpoint=0;
              turnback=false;
              preparestart=true;
              
             }
             if (tsensorPosition<3 && tsensorPosition>-3 && preparestart==true){
              start=true;
              preparestart=false;
             }

        }
      break;
    
    }
  }





@Override
public void teleopInit(){
drivetrain.setbrake(true);
} 

@Override
  public void teleopPeriodic() {
<<<<<<< HEAD
    // DataLogManager.start();
    
    //drivetrain.setbrake(false);
    speed_selected = speed_chooser.getSelected();
     SmartDashboard.putString("Speed Chosen", speed_selected);

      if (controller2.getRightStickButton()){
=======
   
		
   if (controller2.getXButton()){
>>>>>>> f3875e76b41260b89cb7ceed590dfb9164ac2482
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
<<<<<<< HEAD
            drivetrain.setbrake(true);
          }
          else {
            drivetrain.setbrake(true);
            drivetrain.tankDrive(right.getY() * speedMult, left.getY() * speedMult,false);
              drivetrain.mywatchdog(); 
          }


   
      //Relese Hand
      if (controller2.getXButton()) {
=======
           }
           else {
            drivetrain.tankDrive(right.getY() * speedMult, left.getY() * speedMult,false);
            drivetrain.tankwatchdog(); 
           }
        


  
          

          
      // // Hand controlled by left and right triggers
      if (controller2.getPOV()==90) {
>>>>>>> f3875e76b41260b89cb7ceed590dfb9164ac2482
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


<<<<<<< HEAD
=======






>>>>>>> f3875e76b41260b89cb7ceed590dfb9164ac2482
}