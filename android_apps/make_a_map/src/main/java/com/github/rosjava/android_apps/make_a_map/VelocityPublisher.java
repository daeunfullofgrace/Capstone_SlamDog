package com.github.rosjava.android_apps.make_a_map;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.Timer;
import java.util.TimerTask;

import geometry_msgs.Twist;
import geometry_msgs.Vector3;

class VelocityPublisher extends AbstractNodeMain {

    private double velY = 0;
    private double velX = 0;
    private double velZ = 0;
    private double angZ = 0;

    private Publisher<Twist> cmdVelPublisher;
    private Twist cmd_vel;
    private final double MAX_VEL = 0.2;
    private final double MAX_ANG_VEL = 0.3;
    private Timer timer;

    private int typeL = -1;
    private int typeA = -1;

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        this.cmdVelPublisher = connectedNode.newPublisher("/cmd_vel", Twist._TYPE);
        this.cmd_vel = connectedNode.getTopicMessageFactory().newFromType(Twist._TYPE);

        Subscriber<Twist> velocitySub = connectedNode.newSubscriber("cmd_vel", Twist._TYPE);
        velocitySub.addMessageListener(new MessageListener<Twist>() {
            @Override
            public void onNewMessage(Twist msg) {
                velX = msg.getLinear().getX();
                velY = msg.getLinear().getY();
                velZ = msg.getLinear().getZ();

                angZ = msg.getAngular().getZ();

//                Log.d("cmd_vel", velX + "  " + velY);
            }
        });

        timer = new Timer();

        TimerTask TT = new TimerTask() {
            @Override
            public void run() {
                cmdVelPublisher.publish(cmd_vel);
            }

        };

        timer.schedule(TT, 0, 100);
    }

    private double max(double d) {
        if(d >= MAX_VEL) {
            return MAX_VEL;
        } else if(d <= -MAX_VEL) {
            return -MAX_VEL;
        } else {
            return d;
        }
    }

    private double maxAngVel(double d) {
        if(d >= MAX_ANG_VEL) {
            return MAX_ANG_VEL;
        } else if(d <= -MAX_ANG_VEL) {
            return -MAX_ANG_VEL;
        } else {
            return d;
        }
    }

//    public void setForward() {
//        typeL = 0;
//    }
//
//    public void setBackward() {
//        typeL = 2;
//    }
//
//    public void setLeft() {
//        typeL = 1;
//        typeA = 0;
//    }
//
//    public void setRight() {
//        typeL = 3;
//        typeA = 1;
//    }
//
//    public void setStop() {
//        typeL = 4;
//        typeA = 2;
//    }


    public void publishVelocity(int type){
        Vector3 vel = this.cmd_vel.getLinear();
        Vector3 ang = this.cmd_vel.getAngular();

        double x = velX;
        double z = velZ;
        double ang_z = angZ;

        switch (type) {
            case 0:
                x += 0.01;
                break;

            case 1:
                z += 0.05;
                ang_z += 0.05;
                break;

            case 2:
                x -= 0.01;
                break;

            case 3:
                z -= 0.05;
                ang_z -= 0.05;
                break;

            case 4:
                x = 0.0;
                z = 0.0;
                ang_z = 0.0;
                break;

            default:
                break;
        }

        vel.setX(max(x));
        vel.setZ(maxAngVel(z));
        ang.setZ(maxAngVel(ang_z));

        this.cmd_vel.setLinear(vel);
        this.cmd_vel.setAngular(ang);

//        this.cmdVelPublisher.publish(this.cmd_vel);
    }



//    public void publish() {
//        this.cmd_vel.setLinear(setLinear());
//        this.cmd_vel.setAngular(setAngular());
//
//        this.cmdVelPublisher.publish(this.cmd_vel);
//    }

    public double getVel(){
        return this.cmd_vel.getLinear().getX();
    }

    public double getAngVel() {
        return this.cmd_vel.getAngular().getZ();
    }

}
