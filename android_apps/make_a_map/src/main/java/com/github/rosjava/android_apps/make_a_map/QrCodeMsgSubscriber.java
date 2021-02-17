package com.github.rosjava.android_apps.make_a_map;

import android.util.Log;
import android.widget.TextView;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

public class QrCodeMsgSubscriber extends AbstractNodeMain {

    private String qrcodeMsg = "";
    private TextView msgView;

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        Subscriber<std_msgs.String> msgSub = connectedNode.newSubscriber("qrcode_scan", std_msgs.String._TYPE);
        msgSub.addMessageListener(new MessageListener<std_msgs.String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                qrcodeMsg = message.getData();

                Log.d("input data : ", message.getData());
//                Toast.makeText(context, message.getData(), Toast.LENGTH_SHORT).show();
            }
        });
    }

    public String getMsg() {
        String tmp = qrcodeMsg;
        qrcodeMsg = "";
        return tmp;
    }
}
