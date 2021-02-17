package com.github.rosjava.android_apps.make_a_map.ros_custom_service;

public interface SaveMapRequest extends org.ros.internal.message.Message{

    java.lang.String _TYPE = "world_canvas_msgs/SaveMapRequest";
    java.lang.String _DEFINITION = "# Service used to save a given map from the database to the /map topic.\n\nstring map_id\n";

    java.lang.String setMapName();

    void setMapName(java.lang.String s);

}
