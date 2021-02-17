package com.github.rosjava.android_apps.make_a_map.ros_msgs_tutorial;

public interface srvTutorialRequest extends org.ros.internal.message.Message {
    java.lang.String _TYPE = "com.github.rosjava.android_apps.make_a_map/ros_msgs_tutorial/srvTutorialRequest";
    java.lang.String _DEFINITION = "int64 a\nint64 b\n";

    long getA();

    void setA(long l);

    long getB();

    void setB(long l);
}
