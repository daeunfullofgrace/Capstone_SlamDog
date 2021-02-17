package com.github.rosjava.android_apps.make_a_map.ros_msgs_tutorial;

public interface srvTutorialResponse extends org.ros.internal.message.Message  {
    java.lang.String _TYPE = "com.github.rosjava.android_apps.make_a_map/ros_msgs_tutorial/srvTutorialResponse";
    java.lang.String _DEFINITION = "int64 result";

    long getSum();

    void setSum(long l);
}
