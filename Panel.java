import javax.swing.JPanel;
import javax.swing.JFrame;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Color;
import java.time.Instant;
import java.time.Duration;
import java.awt.MouseInfo;
import java.awt.Point;
public class Panel extends JPanel{
   int cW = 800, cH = 800;
   double dt = 0.01666;
   Instant start = null;
   Instant end = null;
   double mul = 1;
   double time = 0;
   int mouseX = 0, mouseY = 0;
   Rocket[] rocket = new Rocket[15];
   double[] dist = new double[rocket.length];
   //double[] times = new double[rocket.length];
   int[] count = new int[rocket.length];
   double[] x = new double[rocket.length], y = new double[rocket.length];
   JFrame frame;
   int noTries = 100;
   boolean moving = true;
   
   boolean targetPractice = true;
   boolean movingFormation = false;
   
   
   
   public static void main(String[] args){
      Panel p = new Panel();
   }
   public void getMouseLocation(){
      Point p = MouseInfo.getPointerInfo().getLocation();
      Point c = this.getLocationOnScreen();
      mouseX = p.x-c.x;
      mouseY = p.y-c.y;
   }
   Panel(){
      for(int i = 0; i < rocket.length; i++){
         rocket[i] = new Rocket(cW, cH, (cW/2)*Math.random()+cW/2,(cH/2)*Math.random()+cH/2);
         x[i] = (cW-50)*Math.random()+25;
         y[i] = (cH-100)*Math.random()+50;
         dist[i] = Math.sqrt((x[i]-rocket[i].x)*(x[i]-rocket[i].x)+(y[i]-rocket[i].y)*(y[i]-rocket[i].y));
         rocket[i].vx = 0;
         rocket[i].moving = moving;
      }
      frame = new JFrame("PID");
      frame.add(this);
      frame.setPreferredSize(new Dimension(cW, cH));
      frame.pack();
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      start = Instant.now();
      frame.setVisible(true);
      while(true){
         try{
            Thread.sleep(1);
         } catch(Exception e){}
         repaint();
      }
   }
   public void paint(Graphics g){
      super.paint(g);
      if(Math.random() < 0.0002){
          //mul *= -1;
      }
      //System.out.println(mul);
      boolean bool = true;
      cW = frame.getWidth();
      cH = frame.getHeight();
      for(int i = 0; i < rocket.length; i++){
      if(count[i] < noTries){
         bool = false;
      }
      rocket[i].cW = cW;
      rocket[i].cH = cH;
      
      if(movingFormation){
         //Two example presets x and y
         x[i] = ((cW-100)/2*Math.cos((time+i)/30)*Math.sin((3*(time+i))/30)+cW/2);
         y[i] = ((cH-100)/2*Math.sin((time+i)/30)*Math.sin((3*(time+i))/30)+cH/2);
         //x[i] = cW/2*Math.exp(-Math.sin((time+i)/10)*Math.sin((time+i)/10));
         //y[i] = cH/2+cH/2*Math.sin((time+i)/10)*Math.sin((time+i)/10)*Math.sin((time+i)/10);
      }
      //int prevMouseX = mouseX;
      //int prevMouseY = mouseY;
      //getMouseLocation();
      //x[i] = (double)(mouseX+prevMouseX)/2;
      //y[i] = (double)(cH-mouseY+cH-prevMouseY)/2;
      
      
      //Rocket has to stop at target
      if(targetPractice && (Math.abs(rocket[i].javaX - x[i]) < 3 && Math.abs(rocket[i].javaY - (cH-y[i])) < 3 && Math.sqrt(rocket[i].vx*rocket[i].vx+rocket[i].vy*rocket[i].vy) < 2)){
         double subx = x[i];
         double suby = y[i];
         rocket[i].newTarget = true;
         x[i] = (cW-100)*Math.random()+50;
         y[i] = (cH-200)*Math.random()+100;
         count[i]++;
         if(count[i] == noTries){
            dist[i] /= time;
            System.out.println(dist[i]+","+i);
         } else if(count[i] < noTries){
            dist[i] += Math.sqrt((subx-x[i])*(subx-x[i])+(suby-y[i])*(suby-y[i]));
         }
         //System.out.println(x+","+y);
      } else {
         moving = false;
      }
      rocket[i].display(g);
      end = Instant.now();
      dt = 0;//(double)Duration.between(start,end).toMillis()/1000;//0.002;
      if(dt == 0){
         dt = .01;
      }
      start = Instant.now();
      //System.out.println(dt);
      rocket[i].calculatePID(x[i],y[i],dt);
      rocket[i].timeStep(dt);
      //System.out.println(dt);
      g.setColor(Color.RED);
      g.fillOval((int)(x[i]-5),(int)(cH-y[i]-5),10,10);
      g.setColor(Color.BLUE);
      //g.fillOval((int)(rocket[i].javaX-5),(int)(rocket[i].javaY-5),10,10);
      g.setColor(Color.GRAY);
      //g.fillOval((int)(rocket[i].javaX+rocket[i].targetax-5),(int)(rocket[i].javaY-rocket[i].targetay-5),10,10);
      g.setColor(Color.GREEN);
      //g.fillOval((int)(rocket[i].javaX+rocket[i].targetvx-5),(int)(rocket[i].javaY-rocket[i].targetvy-5),10,10);
      g.setColor(Color.ORANGE);
      //g.fillOval((int)(rocket[i].javaX+rocket[i].targetvx+rocket[i].dotvx-5),(int)(rocket[i].javaY-rocket[i].targetvy-rocket[i].dotvy-5),10,10);
      /*
      if(rocket.vx > 0){
         g.fillRect((int)rocket.javaX,(int)rocket.javaY-5,(int)rocket.vx,10);
      } else {
         g.fillRect((int)(rocket.javaX+rocket.vx),(int)rocket.javaY-5,(int)Math.abs(rocket.vx),10);
      }
      g.setColor(Color.GRAY);
      g.fillOval((int)(rocket.javaX+rocket.targetax-5),(int)(rocket.javaY-rocket.targetay-5),10,10);
      if(rocket.targetvx > 0){
         g.fillRect((int)rocket.javaX,(int)rocket.javaY-10,(int)rocket.targetvx,10);
      } else {
         g.fillRect((int)(rocket.javaX+rocket.targetvx),(int)rocket.javaY-10,(int)Math.abs(rocket.targetvx),10);
      }
      g.setColor(Color.GRAY);*/
      /*
      g.setColor(Color.MAGENTA);
      if(rocket.ax > 0){
         g.fillRect((int)rocket.javaX,(int)rocket.javaY-5,(int)rocket.ax,10);
      } else {
         g.fillRect((int)(rocket.javaX+rocket.ax),(int)rocket.javaY-5,(int)Math.abs(rocket.ax),10);
      }
      System.out.println(rocket.vx);*/
      g.setColor(Color.BLACK);
      }
      time += mul*dt;
      if(bool){
         System.exit(1);
      }
   }
}