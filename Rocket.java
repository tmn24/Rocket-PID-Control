import java.awt.Polygon;
import java.awt.Graphics;
import java.awt.Color;
public class Rocket{
   double x=0, y=0, vx=0, vy=0, ax=0, ay=0, theta=0, thetaV=0, thetaA=0, mass = 1, momentOfInertia = 1;
   int javaX, javaY;
   double targetax, targetay;
   double javaTheta;
   double maxForce = 50;
   double maxTorque = 2;
   double rocketForce = 0;
   double targetvx = 0,targetvy = 0, targetThetaV = 0;
   double dotvx = 0; double dotvy = 0;
   double dotax = 0; double dotay = 0;
   int width = 30, height = 30;
   double P,I,D;
   double prevXError = 0, prevYError = 0, prevThetaError = 0, prevVXError = 0, prevVYError = 0, prevThetaVError = 0;
   double sumvx = 0, sumvy = 0, sumax = 0, sumay = 0, sumthetav = 0, sumthetaA;
   int cW = 0, cH = 0;
   boolean moving = false;
   boolean newTarget = false;
   double prevX = 0;
   double prevVX = 0;
   double prevY = 0; 
   double prevVY = 0;
   
   double gravity = 0;
   
   Rocket(int cW, int cH, double x, double y){
      this.cW = cW;
      this.cH = cH;
      this.x = x;
      this.y = y;
   }
   public void timeStep(double dt){
      x += vx*dt+ax*dt*dt*0.5;
      y += vy*dt+ay*dt*dt*0.5;
      vx += ax*dt;
      vy += ay*dt;
      theta += thetaV*dt+thetaA*dt*dt*0.5;
      thetaV += thetaA*dt;
   }
   public void toJava(){
      javaX = (int)x;
      javaY = (int)(cH - y);
      javaTheta = -theta;
   }
  /* public void calculatePID(double x, double dt){
      double error = (x-this.x);
      if(prevXError == 0){
         prevXError = (x-this.x);
      }
      P = 1;
      I = .001;
      D = 1;
      sum += error*dt;
      double output = P*error+I*sum+D*(error-prevXError)/dt;
      targetvx = output;
      error = targetvx-vx;
      if(prevVXError == 0){
         prevVXError = (targetvx- vx);
      }
      P = .05;
      I = 0;
      D = 0;
      if(output > 1){
         output = 1;
      } else if(output < -1){
         output = -1;
      }
      sum += error*dt;
      output = P*error+I*sum+D*(error-prevVXError)/dt;
      ax = output*maxForce/mass;
      //Save for after method is over
      prevXError = (x-this.x);
      prevVXError = (targetvx-vx);
   }*/
   public void calculatePID(double x, double y, double dt){
      double dCoef = 1;
      double accelTime = 3;
      //double dotvx = 0;
      //double dotvy = 0;
      if(newTarget){
         newTarget = false;
          sumvx = 0; sumvy = 0; sumax = 0; sumay = 0; sumthetav = 0; sumthetaA = 0;
      }
      if(moving){
         dCoef = 0;
         if(prevX == 0){
            prevX = x;
         }
         if(prevY == 0){
            prevY = y;
         }
         double subx = x;
         double suby = y;
         double subvx = dotvx;
         double subvy = dotvy;
         dotvx = (x-prevX)/dt;
         dotax = 0*(dotvx-prevVX)/dt;
         //System.out.println(prevX+","+x+"|");
         dotvy = (y-prevY)/dt;
         dotay = 0*(dotvy-prevVY)/dt;
         //System.out.println(dotvx);
         prevX = subx;
         prevY = suby;
         prevVX = subvx;
         prevVY = subvy;
         //x += dotvx*1;
         //y += dotvy*1;
      }
      double error = (x-this.x);
      if(prevXError == 0){
         prevXError = (x-this.x);
      }
      P = .3;
      I = 0;
      D = 0;//0.05*dCoef;
      sumvx += error*dt;
      double output = P*error+I*sumvx+D*(error-prevXError)/dt;
      targetvx = output;
      if(prevVXError == 0){
         prevVXError = (targetvx-vx);
      }
      
      error = (y-this.y);
      if(prevYError == 0){
         prevYError = (y-this.y);
      }
      P = .3;
      I = 0;
      D = 0;//0.05*dCoef;
      sumvy += error*dt;
      output = P*error+I*sumvy+D*(error-prevYError)/dt;
      targetvy = output;
      if(prevVYError == 0){
         prevVYError = (targetvy-vy);
      }
      
      //System.out.println(dotvx);
      //System.out.println(targetvx);
      error = targetvx+dotvx-vx;
      if(targetvx > maxForce/mass*accelTime){
         error = maxForce/mass*accelTime+dotvx-vx;
      } else if(targetvx < -maxForce/mass*accelTime){
         error = -maxForce/mass*accelTime+dotvx-vx;
      }
      targetvx = error;
      //System.out.println(error);
      //System.out.println(error+","+dotvx);
      P = .2;
      I = 0;
      D = 0.05*dCoef;//.0005*dCoef;
      
      //P = 0.01 I = 0.0001 D = 0;
      sumax += error*dt;
      output = P*error+I*sumax+D*(error-prevVXError)/dt;
      if(output > 1){
         output = 1;
      } else if(output < -1){
         output = -1;
      }
      targetax = output*maxForce/mass+dotay;
      
      
      error = targetvy+dotvy-vy;
      if(targetvy > maxForce/mass*accelTime){
         error = maxForce/mass*accelTime+dotvy-vy;
      } else if(targetvy < -maxForce/mass*accelTime){
         error = -maxForce/mass*accelTime+dotvy-vy;
      }
      targetvy = error;
      P = .2;
      I = 0.000;
      D = 0.05*dCoef;//.0005*dCoef;
      sumay += error*dt;
      output = P*error+I*sumay+D*(error-prevVYError)/dt;
      //System.out.println(output);
      if(output > 1){
         output = 1;
      } else if(output < -1){
         output = -1;
      }
      targetay = output*maxForce/mass+gravity+dotax;
      
      
      double targetTheta = Math.atan(targetay/targetax);
      if(targetax < 0){
         targetTheta += Math.PI;
      }
      while(targetTheta - theta > Math.PI){
         targetTheta -= Math.PI*2;
      }
      while(targetTheta - theta < -Math.PI){
         targetTheta += Math.PI*2;
      }
      //System.out.println(targetvx+","+targetvy);
      //System.out.println(theta+"|||");
      //System.out.println(targetTheta+">");
      error = (targetTheta-theta);
      if(prevThetaError == 0){
         prevThetaError = error;
      }
      P = 3;//.75
      I = 0;
      D = 0;//.15*dCoef;
      sumthetav += error*dt;
      output = P*error+I*sumthetav+D*(error-prevThetaError)/dt;
      targetThetaV = output;
      prevThetaError = (targetTheta-theta);
      //System.out.println(targetThetaV+">");
      
      
      error = targetThetaV-thetaV;
      if(prevThetaVError == 0){
         prevThetaVError = error;
      }
      //System.out.println(theta+","+targetTheta+","+targetThetaV+","+thetaV);
      P = 1;//1.5
      I = 0;
      D = .8;//0.15*dCoef;
      sumthetaA += error*dt;
      output = P*error+I*sumthetaA+D*(error-prevThetaVError)/dt;
      if(output > 1){
         output = 1;
      } else if(output < -1){
         output = -1;
      }
      thetaA = output*maxTorque/momentOfInertia;
      //System.out.println(thetaV+"|");
      //System.out.println(thetaA+"|");
      double a = Math.sqrt(targetax*targetax+targetay*targetay);
      ax = a*Math.cos(theta)*Math.pow(Math.abs(Math.PI-Math.abs(targetTheta-theta))/Math.PI,8);
      ay = a*Math.sin(theta)*Math.pow(Math.abs(Math.PI-Math.abs(targetTheta-theta))/Math.PI,8)-gravity; 
      rocketForce = a/maxForce*Math.pow(Math.abs(Math.PI-Math.abs(targetTheta-theta))/Math.PI,8);
      prevThetaVError = error;
      //Save for after method is over
      prevYError = (y-this.y);
      prevVYError = (targetvy-vy);
      //Save for after method is over
      prevXError = (x-this.x);
      prevVXError = (targetvx-vx);
   }
   public void display(Graphics g){
      toJava();
      double angle = Math.atan(height/width);
      double diagonal = Math.sqrt(height*height*0.25+width*width*0.25);
      g.setColor(Color.RED);
      g.fillPolygon(new Polygon(new int[]{(int)(javaX+(-width/2-100*rocketForce)*Math.cos(javaTheta)),(int)(javaX+diagonal*Math.cos(javaTheta+Math.PI-angle)),(int)(javaX+diagonal*Math.cos(javaTheta+Math.PI+angle))},new int[]{(int)(javaY-(height/2+100*rocketForce)*Math.sin(javaTheta)),(int)(javaY+diagonal*Math.sin(javaTheta+Math.PI-angle)),(int)(javaY+diagonal*Math.sin(javaTheta+Math.PI+angle))},3));
      g.setColor(Color.BLACK);
      g.fillPolygon(new Polygon(new int[]{(int)(javaX+40*Math.cos(javaTheta)),(int)(javaX+diagonal*Math.cos(javaTheta+angle)),(int)(javaX+diagonal*Math.cos(javaTheta+Math.PI-angle)),(int)(javaX+diagonal*Math.cos(javaTheta+Math.PI+angle)),(int)(javaX+diagonal*Math.cos(javaTheta-angle))},new int[]{(int)(javaY+40*Math.sin(javaTheta)),(int)(javaY+diagonal*Math.sin(javaTheta+angle)),(int)(javaY+diagonal*Math.sin(javaTheta+Math.PI-angle)),(int)(javaY+diagonal*Math.sin(javaTheta+Math.PI+angle)),(int)(javaY+diagonal*Math.sin(javaTheta-angle))},5));
   }
}