/*
 * Copyright (C) 2013 OSRF.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.rosjava.android_apps.make_a_map;

import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.Vibrator;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;
import com.google.common.collect.Lists;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.view.RosImageView;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.layer.CameraControlListener;
import org.ros.android.view.visualization.layer.LaserScanLayer;
import org.ros.android.view.visualization.layer.Layer;
import org.ros.android.view.visualization.layer.OccupancyGridLayer;
import org.ros.android.view.visualization.layer.RobotLayer;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.time.NtpTimeProvider;
import org.ros.time.TimeProvider;
import org.ros.time.WallTimeProvider;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

/**
 * @author murase@jsk.imi.i.u-tokyo.ac.jp (Kazuto Murase)
 */
public class MainActivity extends RosAppActivity {

    private static final int NAME_MAP_DIALOG_ID = 0;

	private RosImageView<sensor_msgs.CompressedImage> cameraView;
//	private VirtualJoystickView virtualJoystickView;

	private Button buttonT;
	private Button buttonL;
	private Button buttonR;
	private Button buttonC;
	private Button buttonB;
	private RelativeLayout controllerView;
	private TextView velocityView;
	private TextView msgText;

	private VisualizationView mapView;
	private ViewGroup mainLayout;
	private ViewGroup sideLayout;
	private ImageButton refreshButton;
	private ImageButton saveButton;
	private Button backButton;
	private NodeMainExecutor nodeMainExecutor;
	private NodeConfiguration nodeConfiguration;
	private ProgressDialog waitingDialog;
	private AlertDialog notiDialog;
	private VelocityPublisher publisher;
	private QrCodeMsgSubscriber msgSubscriber;

    private OccupancyGridLayer occupancyGridLayer = null;
    private LaserScanLayer laserScanLayer = null;
    private RobotLayer robotLayer = null;
    private Timer timer;
    private TimerTask TT;

    Client client;

	public MainActivity() {
		super("Make a map", "Make a map");
	}

	@SuppressWarnings("unchecked")
	@Override
	public void onCreate(Bundle savedInstanceState) {

		String defaultRobotName = getString(R.string.default_robot);
		String defaultAppName = getString(R.string.default_app);
        setDefaultMasterName(defaultRobotName);
		setDefaultAppName(defaultAppName);
		setDashboardResource(R.id.top_bar);
		setMainWindowResource(R.layout.main);

		super.onCreate(savedInstanceState);

		cameraView = (RosImageView<sensor_msgs.CompressedImage>) findViewById(R.id.image);
		cameraView.setMessageType(sensor_msgs.CompressedImage._TYPE);
		cameraView.setMessageToBitmapCallable(new BitmapFromCompressedImage());
//		virtualJoystickView = (VirtualJoystickView) findViewById(R.id.virtual_joystick);
		refreshButton = (ImageButton) findViewById(R.id.refresh_button);
		saveButton = (ImageButton) findViewById(R.id.save_map);
		backButton = (Button) findViewById(R.id.back_button);
		velocityView = findViewById(R.id.vel_view);
		msgText = findViewById(R.id.msg_data);

		buttonT = findViewById(R.id.btn_top);
		buttonB = findViewById(R.id.btn_bottom);
		buttonC = findViewById(R.id.btn_center);
		buttonR = findViewById(R.id.btn_right);
		buttonL = findViewById(R.id.btn_left);

        mapView = (VisualizationView) findViewById(R.id.map_view);
        mapView.onCreate(Lists.<Layer>newArrayList());

		mapView.getCamera().jumpToFrame((String) params.get("map_frame", getString(R.string.map_frame)));

		mainLayout = (ViewGroup) findViewById(R.id.main_layout);
		sideLayout = (ViewGroup) findViewById(R.id.side_layout);

        refreshButton.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				// TODO
				Toast.makeText(MainActivity.this, "refreshing map...",
						Toast.LENGTH_SHORT).show();
                mapView.getCamera().jumpToFrame((String) params.get("map_frame", getString(R.string.map_frame)));
			}
		});

		saveButton.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				client = new Client();
				nodeMainExecutor.execute(client, nodeConfiguration.setNodeName("android/ros_msgs_tutorial"));
			}
		});

		backButton.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				onBackPressed();
			}
		});

		buttonT.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				publisher.publishVelocity(0);
				updateVelView();
			}
		});

		buttonB.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				publisher.publishVelocity(2);
				updateVelView();
			}
		});

		buttonL.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				publisher.publishVelocity(1);
				updateVelView();
			}
		});

		buttonR.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				publisher.publishVelocity(3);
				updateVelView();
			}
		});

		buttonC.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				publisher.publishVelocity(4);
				updateVelView();
			}
		});

		timer = new Timer();
		final Vibrator vibrator = (Vibrator)getSystemService(Context.VIBRATOR_SERVICE);

		@SuppressLint("HandlerLeak") final Handler handler = new Handler(){
			public void handleMessage(Message message){
				msgText.setText((String) message.obj);
			}
		};

		TT = new TimerTask() {
			@Override
			public void run() {
				String msg = msgSubscriber.getMsg();
				if(!msg.equals("")) {
//					Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show();
					vibrator.vibrate(1000);
					Message message = Message.obtain();
					message.obj = msg;
					message.setTarget(handler); // Set the Handler
					message.sendToTarget();
				}
			}
		};
	}

	private void updateVelView(){
		velocityView.setText(Math.round(publisher.getVel()*100)/100.0 + "    " + Math.round(publisher.getAngVel()*100)/100.0);
	}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {

		super.init(nodeMainExecutor);
		this.nodeMainExecutor = nodeMainExecutor;

		nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory
				.newNonLoopback().getHostAddress(), getMasterUri());

        String joyTopic = remaps.get(getString(R.string.joystick_topic));
        String camTopic = remaps.get(getString(R.string.camera_topic));

        NameResolver appNameSpace = getMasterNameSpace();
        joyTopic = appNameSpace.resolve(joyTopic).toString();
        camTopic = appNameSpace.resolve(camTopic).toString();
        cameraView.setTopicName(camTopic);

		publisher = new VelocityPublisher();
		msgSubscriber = new QrCodeMsgSubscriber();
		timer.schedule(TT, 0, 100);

		nodeMainExecutor.execute(cameraView,
				nodeConfiguration.setNodeName("android/camera_view"));
		nodeMainExecutor.execute(publisher,
				nodeConfiguration.setNodeName("android/velocity_publisher"));
		nodeMainExecutor.execute(msgSubscriber,
				nodeConfiguration.setNodeName("android/msg_subscriber"));

        ViewControlLayer viewControlLayer = new ViewControlLayer(this,
                nodeMainExecutor.getScheduledExecutorService(), cameraView,
                mapView, mainLayout, sideLayout, params);

        String mapTopic   = remaps.get(getString(R.string.map_topic));
        String scanTopic  = remaps.get(getString(R.string.scan_topic));
        String robotFrame = (String) params.get("robot_frame", getString(R.string.robot_frame));

        occupancyGridLayer = new OccupancyGridLayer("/map");
        laserScanLayer = new LaserScanLayer(appNameSpace.resolve(scanTopic).toString());
        robotLayer = new RobotLayer(robotFrame);

        mapView.addLayer(viewControlLayer);
        mapView.addLayer(occupancyGridLayer);
        mapView.addLayer(laserScanLayer);
        mapView.addLayer(robotLayer);

        mapView.init(nodeMainExecutor);
        viewControlLayer.addListener(new CameraControlListener() {
            @Override
            public void onZoom(float focusX, float focusY, float factor) {}
            @Override
            public void onDoubleTap(float x, float y) {}
            @Override
            public void onTranslate(float distanceX, float distanceY) {}
            @Override
            public void onRotate(float focusX, float focusY, double deltaAngle) {}
        });

		TimeProvider timeProvider = null;
		try {
			NtpTimeProvider ntpTimeProvider = new NtpTimeProvider(
					InetAddressFactory.newFromHostString("pool.ntp.org"),
					nodeMainExecutor.getScheduledExecutorService());
			ntpTimeProvider.startPeriodicUpdates(1, TimeUnit.MINUTES);
			timeProvider = ntpTimeProvider;
		} catch (Throwable t) {
			Log.w("MakeAMap", "Unable to use NTP provider, using Wall Time. Error: " + t.getMessage(), t);
			timeProvider = new WallTimeProvider();
		}
		nodeConfiguration.setTimeProvider(timeProvider);

		nodeMainExecutor.execute(mapView, nodeConfiguration.setNodeName("android/map_view"));
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		menu.add(0, 0, 0, R.string.stop_app);
		return super.onCreateOptionsMenu(menu);
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		super.onOptionsItemSelected(item);
		switch (item.getItemId()) {
		case 0:
			onDestroy();
			break;
		}
		return true;
	}
}