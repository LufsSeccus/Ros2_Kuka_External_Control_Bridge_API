package application;

import javax.inject.Inject;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketTimeoutException;
import java.util.Locale;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;
import com.kuka.task.ITaskLogger;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class UDP_bridge extends RoboticsAPIApplication {

   @Inject private KmpOmniMove kmp;
   @Inject private LBR lbr;
   @Inject private ITaskLogger logger; 
   private volatile int baseTargetReached = 0;
   private DatagramSocket udpSocket;
   private InetAddress ros2Address = null;

   private final int PORT_ROBOT = 30300;  
   private volatile int PORT_CLIENT = 30333;

   private volatile boolean isRunning = true;
   private volatile boolean isApplicationActive = false;
   
   // Protocol tracking
   private long lastRxCounter = -1;
   private long txCounter = 0;
   private boolean lastAppStartSignal = false;
   private long lastVelCommandTime = 0;

   // Non-blocking motion containers
   private Thread telemetryThread; 
   private Thread platformWorkerThread;
   
   private IMotionContainer activeArmMotionContainer = null;
   
   // --- SPEED TRACKING VARIABLES ---
   // Array storing individual speeds for all 7 LBR joints (default 20%)
   private volatile double[] currentArmSpeeds = new double[] {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
   // Scalar value for KMP base speed (default 20%)
   private volatile double currentBaseSpeed = 0.2; 

   private static class PlatformMotionCmd {
       private double dx; 
       private double dy; 
       private double dAlphaDegrees; 
       PlatformMotionCmd(double dx, double dy, double dAlphaDegrees){ 
           this.dx = dx;
           this.dy = dy; 
           this.dAlphaDegrees = dAlphaDegrees;
       }
   }
   
   private final BlockingQueue<PlatformMotionCmd> platformQueue = new LinkedBlockingQueue<PlatformMotionCmd>();

   private class KmpPose {
       double x = 0;
       double y = 0;
       double alpha = 0;

       void update(double dx, double dy, double dAlpha) {
           double radAlpha = Math.toRadians(alpha);
           x += dx * Math.cos(radAlpha) - dy * Math.sin(radAlpha);
           y += dx * Math.sin(radAlpha) + dy * Math.cos(radAlpha);
           alpha += dAlpha;
       }
   }
   private KmpPose kmpPose = new KmpPose();

   @Override
   public void initialize() {
       try {
           udpSocket = new DatagramSocket(PORT_ROBOT);
           ros2Address = null;     
           startPlatformWorker();
           startTelemetryThread();
           logger.info("KMP + LBR UDP Bridge initialized. Awaiting ROS2 handshake...");
       } catch (Exception e) {
           logger.error("Setup failed: " + e.getMessage());
       }
   }
   
   private void startTelemetryThread(){
       telemetryThread = new Thread(new Runnable(){
           @Override
           public void run(){ 
               logger.info("[Telemetry Thread] Started streaming loop at 20Hz");
               while(isRunning){
                   try {
                       if(isApplicationActive && ros2Address!= null ){
                           sendNativeTelemetry();
                       }
                       Thread.sleep(50);
                   }catch(InterruptedException e){
                       Thread.currentThread().interrupt();
                       break;
                   }catch (Exception e){
                       logger.error("[Telemetry Thread] Error: "+ e.getMessage());
                   }
               }
           }
       });
       
       telemetryThread.setName("UDP-Telemetry-Tx");
       telemetryThread.start();
   }
   
   private void startPlatformWorker(){
       platformWorkerThread = new Thread(new Runnable(){
           @Override
           public void run(){
               while(isRunning){
                   try{
                       PlatformMotionCmd cmd = platformQueue.take();
                       baseTargetReached = 0;
                       double dAlpha_rad = Math.toRadians(cmd.dAlphaDegrees);
                       
                       MobilePlatformRelativeMotion relMotion = new MobilePlatformRelativeMotion(cmd.dx, cmd.dy, dAlpha_rad);
                       // Apply dynamically received base speed ratio
                       relMotion.setVelocityRatio(currentBaseSpeed);
                       
                       kmp.move(relMotion);
                       baseTargetReached = 1;
                   }catch (InterruptedException e){
                       Thread.currentThread().interrupt();
                       break;
                   }catch (Exception e){
                       logger.error("Platform execution error: " + e.getMessage());
                   }
               }
           }
       });
       platformWorkerThread.setName("UDP-Platform-Worker");
       platformWorkerThread.start();
   }

   @Override
   public void run() {
       byte[] receiveBuf = new byte[512];

       while (isRunning) {
           try {
               DatagramPacket receivePacket = new DatagramPacket(receiveBuf, receiveBuf.length);
               udpSocket.receive(receivePacket);

               InetAddress senderAddr = receivePacket.getAddress();
               int senderPort = receivePacket.getPort();
               String rawMsg = new String(receivePacket.getData(), 0, receivePacket.getLength(), "UTF-8").trim();

               String[] parts = rawMsg.split(";");
               if (parts.length >= 4) {
                   long rxCounter = Long.parseLong(parts[1]);
                   String command = parts[2].trim();
                   String value = parts[3].trim();

                   boolean isHandshake = command.equalsIgnoreCase("App_Start");
                   boolean isNewSender = (ros2Address == null) || 
                                         (!ros2Address.equals(senderAddr)) || 
                                         (PORT_CLIENT != senderPort);

                   if (isHandshake || isNewSender) {
                       ros2Address = senderAddr;
                       PORT_CLIENT = senderPort;
                       lastRxCounter = -1;
                       logger.info(String.format("ROS2 Session Connected: %s:%d", ros2Address.getHostAddress(), PORT_CLIENT));
                   }

                   if (rxCounter <= lastRxCounter) continue; 
                   lastRxCounter = rxCounter;

                   if (command.equalsIgnoreCase("App_Start")) {
                       boolean currentSignal = Boolean.parseBoolean(value);
                       if (!lastAppStartSignal && currentSignal) {
                           logger.info("Core loop activated via App_Start rising edge. Connected.");
                           isApplicationActive = true;
                       }
                       lastAppStartSignal = currentSignal;
                   } else if (command.equalsIgnoreCase("Set_Shutdown")) {
                       shutdownBridge();
                   } else if (isApplicationActive) {
                       processActiveParameters(command, value);
                   }
               }
           } catch (SocketTimeoutException e) {
               // Normal loop iteration timeout
           } catch (Exception e) {
               logger.error("Socket reading error: " + e.getMessage());
           }
       }
   }

   private void processActiveParameters(String command, String value) {
       try {
           // --- DYNAMIC SPEED CONTROLS ---
           if (command.equalsIgnoreCase("Set_Arm_Speed")) {
               String[] speedVals = value.split(",");
               if (speedVals.length == 7) {
                   for (int i = 0; i < 7; i++) {
                       // Clamp each joint speed safely between 1% and 100%
                       currentArmSpeeds[i] = Math.max(0.01, Math.min(1.0, Double.parseDouble(speedVals[i])));
                   }
               }
               return;
           } 
           else if (command.equalsIgnoreCase("Set_Base_Speed")) {
               // Clamp base speed safely between 1% and 100%
               currentBaseSpeed = Math.max(0.01, Math.min(1.0, Double.parseDouble(value)));
               return;
           }

           // 1. KMP VELOCITY MODE
           if (command.equalsIgnoreCase("Set_Vel")) {
               String[] vel = value.split(",");
               double vx = Double.parseDouble(vel[0]); 
               double vy = Double.parseDouble(vel[1]); 
               double omega = Double.parseDouble(vel[2]);

               long currentTime = System.currentTimeMillis();
               if (lastVelCommandTime == 0) {
                   lastVelCommandTime = currentTime;
                   return;
               }

               double dt = (currentTime - lastVelCommandTime) / 1000.0; 
               lastVelCommandTime = currentTime;
               if (dt <= 0 || dt > 0.5) dt = 0.05;

               double dx = vx * dt; 
               double dy = vy * dt; 
               double dAlphaDegrees = Math.toDegrees(omega * dt);
               executeRelativePlatformMotion(dx, dy, dAlphaDegrees);
           }
           // 2. KMP POSE P2P MODE
           else if (command.equalsIgnoreCase("Set_Pose")) {
               String[] pose = value.split(",");
               double targetX = Double.parseDouble(pose[0]); 
               double targetY = Double.parseDouble(pose[1]); 
               double targetAlpha = Double.parseDouble(pose[2]);

               double dX_world = targetX - kmpPose.x;
               double dY_world = targetY - kmpPose.y;
               double dAlphaDegrees = targetAlpha - kmpPose.alpha;

               double radCurr = Math.toRadians(kmpPose.alpha);
               double dx = dX_world * Math.cos(radCurr) + dY_world * Math.sin(radCurr);
               double dy = -dX_world * Math.sin(radCurr) + dY_world * Math.cos(radCurr);
               kmpPose.update(dx, dy, dAlphaDegrees);
               executeRelativePlatformMotion(dx, dy, dAlphaDegrees);
           }
           // 3. LBR ARM JOINT CONTROL MODE
           else if (command.equalsIgnoreCase("Set_Arm_Joint")) {
               String[] joints = value.split(",");
               if (joints.length == 7) {
                   executeArmJointMotion(
                       Math.toRadians(Double.parseDouble(joints[0])), Math.toRadians(Double.parseDouble(joints[1])),
                       Math.toRadians(Double.parseDouble(joints[2])), Math.toRadians(Double.parseDouble(joints[3])),
                       Math.toRadians(Double.parseDouble(joints[4])), Math.toRadians(Double.parseDouble(joints[5])),
                       Math.toRadians(Double.parseDouble(joints[6]))
                   );
               }
           }
           // 4. LBR ARM CARTESIAN END-EFFECTOR MODE
           else if (command.equalsIgnoreCase("Set_Arm_End_Effector")) {
               String[] pose = value.split(",");
               if (pose.length == 6) {
                   double x_mm = Double.parseDouble(pose[0]) * 1000.0;
                   double y_mm = Double.parseDouble(pose[1]) * 1000.0;
                   double z_mm = Double.parseDouble(pose[2]) * 1000.0;
                   double a_rad = Double.parseDouble(pose[3]);
                   double b_rad = Double.parseDouble(pose[4]);
                   double c_rad = Double.parseDouble(pose[5]);
                   executeCartesianArmMotion(x_mm, y_mm, z_mm, a_rad, b_rad, c_rad);
               }
           }
       } catch (Exception e) {
           logger.error("Command parsing error: " + e.getMessage());
       }
   }

   private void executeRelativePlatformMotion(double dx, double dy, double dAlphaDegrees) {
       baseTargetReached = 0;
       platformQueue.offer(new PlatformMotionCmd (dx, dy, dAlphaDegrees));
   }

   private void executeArmJointMotion(double j1, double j2, double j3, double j4, double j5, double j6, double j7) {
       if (activeArmMotionContainer != null && !activeArmMotionContainer.isFinished()) {
           activeArmMotionContainer.cancel();
       }
       JointPosition targetJoints = new JointPosition(j1, j2, j3, j4, j5, j6, j7);
       PTP ptpMotion = new PTP(targetJoints);
       // Apply dynamically received arm speeds explicitly to all 7 joints
       ptpMotion.setJointVelocityRel(currentArmSpeeds);
       activeArmMotionContainer = lbr.moveAsync(ptpMotion);
   }

   private void executeCartesianArmMotion(double x_mm, double y_mm, double z_mm, double a_rad, double b_rad, double c_rad) {
       if (activeArmMotionContainer != null && !activeArmMotionContainer.isFinished()) {
           activeArmMotionContainer.cancel();
       }
       com.kuka.roboticsAPI.geometricModel.Frame targetFrame = 
           new com.kuka.roboticsAPI.geometricModel.Frame(lbr.getRootFrame(), x_mm, y_mm, z_mm, a_rad, b_rad, c_rad);

       PTP ptpMotion = com.kuka.roboticsAPI.motionModel.BasicMotions.ptp(targetFrame);
       // Apply dynamically received arm speeds explicitly to all 7 joints
       ptpMotion.setJointVelocityRel(currentArmSpeeds);
       activeArmMotionContainer = lbr.moveAsync(ptpMotion);
   }

   private void sendNativeTelemetry() {
       if (ros2Address == null) return;
       try {
           long currentTime = System.currentTimeMillis();
           txCounter++;

           // 1. KMP Base Pose Payload
           String posePayload = String.format(Locale.US, "%.3f,%.3f,%.3f,%d", kmpPose.x, kmpPose.y, kmpPose.alpha, baseTargetReached);

           // 2. LBR Arm Joint Positions
           JointPosition currentJoints = lbr.getCurrentJointPosition();
           StringBuilder armPayloadBuilder = new StringBuilder();
           for (int i = 0; i < 7; i++) {
               double deg = Math.toDegrees(currentJoints.get(i));
               armPayloadBuilder.append(String.format(Locale.US, "%.2f", deg));
               if (i < 6) armPayloadBuilder.append(",");
           }
           String armPayload = armPayloadBuilder.toString();

           // 3. LBR End-Effector Cartesian Position
           com.kuka.roboticsAPI.geometricModel.Frame eeFrame = lbr.getCurrentCartesianPosition(lbr.getFlange());
           String eePayload = String.format(Locale.US, "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f",
                   eeFrame.getX(), eeFrame.getY(), eeFrame.getZ(),
                   eeFrame.getAlphaRad(), eeFrame.getBetaRad(), eeFrame.getGammaRad());

           // 4. Format complete packet
           String stateMsg = String.format(Locale.US, "%d;0;%d;%s;%s;%s", 
                   currentTime, txCounter, posePayload, armPayload, eePayload);

           byte[] data = stateMsg.getBytes("UTF-8");
           DatagramPacket packet = new DatagramPacket(data, data.length, ros2Address, PORT_CLIENT);
           udpSocket.send(packet);
       } catch (Exception e) {
           logger.error("Outbound telemetry error: " + e.getMessage());
       }
   }

   private void shutdownBridge() {
       isRunning = false;
       isApplicationActive = false;
       platformQueue.clear();
       
       if (telemetryThread != null && telemetryThread.isAlive()) {
           telemetryThread.interrupt();
       }
       if (platformWorkerThread != null && platformWorkerThread.isAlive()) {
           platformWorkerThread.interrupt();
       }
       if (activeArmMotionContainer != null && !activeArmMotionContainer.isFinished()) {
           activeArmMotionContainer.cancel();
       }
       ros2Address = null;
       lastRxCounter = -1;
   }

   @Override
   public void dispose() {
       shutdownBridge();
       if (udpSocket != null && !udpSocket.isClosed()) {
           udpSocket.close();
       }
       super.dispose();
   }
}