package com.github.rosjava.android_apps.teleop;

import android.util.Log;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import geometry_msgs.Twist;
import geometry_msgs.Vector3;

class VelocitySubscriber extends AbstractNodeMain {

    private double velY = 0;
    private double velX = 0;
    private Vector3 ang;
    private Publisher<Twist> cmdVelPublisher;
    private geometry_msgs.Twist cmd_vel;

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        this.cmdVelPublisher = connectedNode.newPublisher("/cmd_vel", geometry_msgs.Twist._TYPE);
        this.cmd_vel = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Twist._TYPE);

        Subscriber<Twist> velocitySub = connectedNode.newSubscriber("cmd_vel", Twist._TYPE);
        velocitySub.addMessageListener(new MessageListener<Twist>() {
            @Override
            public void onNewMessage(Twist msg) {
                velX = msg.getLinear().getX();
                velY = msg.getLinear().getY();

                ang = msg.getAngular();

                if(Math.abs(velX) >= 0.5 && Math.abs(velY) <= 0.5) {
                    publishVelocity(max(velX), velY);
                } else if(Math.abs(velX) <= 0.5 && Math.abs(velY) >= 0.5) {
                    publishVelocity(velX, max(velY));
                } else if(Math.abs(velX) >= 0.5 && Math.abs(velY) >= 0.5) {
                    publishVelocity(max(velX), max(velY));
                }

                Log.d("cmd_vel", String.valueOf(velX));
            }
        });
    }

    private double max(double d) {
        if(d >= 0.5){
            return 0.5;
        } else if(d <= -0.5) {
            return -0.5;
        } else {
            return d;
        }
    }

    private void publishVelocity(double x, double y){
        Vector3 vel = this.cmd_vel.getLinear() ;

        vel.setX(x);
        vel.setY(y);

        this.cmd_vel.setLinear(vel);
        this.cmd_vel.setAngular(ang);

        this.cmdVelPublisher.publish(this.cmd_vel);
    }
}