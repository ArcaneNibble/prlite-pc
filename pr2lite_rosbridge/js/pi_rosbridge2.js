/*  pi_rosbridge2.js - Version 1.0 2012-09-22

    An HTML5/rosbridge script to teleop and monitor a ROS robot

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
*/

// Get the current hostname
thisHostName = document.location.hostname;

// If the rosbridge server is running on the webserver host, then this will work.
var rosbridgeHost = thisHostName;

// Or, if you want to prompt for the rosbridge host IP use this instead
//var rosbridgeHost = prompt("IP Address of rosbridge Server:", "192.168.1.2");

// Or, set it manually like this.
//var rosbridgeHost =  "192.168.1.9";

var rosbridgePort = "9090";
var mjpegPort = "8080";

// Get the current host and port to be used with the map URL
thisHost = document.location.host;

// Are we on a touch device?
var is_touch_device = 'ontouchstart' in document.documentElement;

// Video parameters
var videoImage = new Image();
//var videoTopic = "/kinect/rgb/image_color_throttle";
var videoTopic = "/kinect/rgb/image_color";
var videoWidth = "320";
var videoHeight = "240";
var screenWidth = "1280";
var videoFPS = 15;
if (is_touch_device) var videoScale = 1.4;
else var videoScale = 2.0;
var videoHandle = null;
var pubHandle = null;
var cameraInfo = "/kinect/rgb/cameraInfo";
var roiTopic = "/roi";

// Rate for the main ROS publisher loop
var rate = 5;

// Base control parameters
var cmdVelTopic = "/cmd_vel";
var defaultMaxLinearSpeed = 0.2;
var defaultMaxAngularSpeed = 1.2;
var maxLinearSpeed = defaultMaxLinearSpeed;
var maxAngularSpeed = defaultMaxAngularSpeed;
var vx = 0;
var vz = 0;
var lastVx = 0;
var lastVz = 0;
var vx_key_increment = 0.02;
var vz_key_increment = 0.05;
var moveBaseTopic = "/move_base_simple/goal";
var initialPoseTopic = "/initialpose";
var amclPoseTopic = "/amcl_pose";
var moveBaseCancelTopic = "/move_base/cancel";
var mapMetadataTopic = "/map_metadata";
var odomTopic = "/odom";
var usingAMCL = false;
var firstSetPose = true;
var firstMoveBase = true;
var firstMoveBaseCancel = true;

// Laser scan topics
var baseLaserScanTopic = "/scan"
var obstacleLaserTopic = "/base_laser_obstacle";
var obstacleDetected = false;
var baseLaserScanRanges;
var baseLaserScanArray;

// Servo parameters
var jointStatesTopic = "/joint_states";
var headPanPositionTopic = "/head_pan_joint/command";
var headTiltPositionTopic = "/head_tilt_joint/command";
var maxHeadPanPosition = 2.5;
var maxHeadTiltPosition = 1.5;
var headPanPosition = 0;
var headTiltPosition = 0;

var headPanSpeedService = '/head_pan_joint/set_speed';
var headTiltSpeedService = '/head_tilt_joint/set_speed';
var maxHeadPanSpeed = 1.5;
var maxHeadTiltSpeed = 1.0;
var defaultHeadPanSpeed = 0.45;
var defaultHeadTiltSpeed = 0.3;
var headPanSpeed = 0;
var headTiltSpeed = 0;
var headPanTorqueService = "/head_pan_joint/torque_enable";
var headTiltTorqueService = "/head_tilt_joint/torque_enable";

var headPanTempTopic = "/head_pan_joint/state";
var headTiltTempTopic = "/head_tilt_joint/state";

var servoSpeedControl = false
var relaxServos = false;
var centerServos = false;
var servosRelaxed = false;
var lockServos = false;

// Battery topics
var laptopBatteryTopic = "/laptop_charge";
var robotBatteryTopic = "/serializer/sensors";
var turtleBotBatteryTopic = "/turtlebot_node/sensor_state";
var turtleBot = false;

// Map parameters
var mapWidth;
var mapHeight;
var mapResolution;
var mapOriginX;
var mapOriginY;
var mapFile;

// Robot position from /odom
var odomPositionX;
var odomPositionY;

// Robot position from /amcl_pose
var amclPositionX;
var amclPositionY;

// Record the start and end coordinates of a goal pose arrow
var goalPoseStartX;
var goalPoseStartY;
var goalPoseEndX;
var goalPoseEndY;

// Track joint names, positions and velocities
var jointNames;
var jointPos;
var jointVel;

// Track the size and position of the region of interest
var roiOffsetX ;
var roiOffsetY;
var roiWidth;
var roiHeight;
var roiMarker = new Kinetic.Rect({
    x: 0,
    y: 0,
    width: 0,
    height: 0,
    stroke: "#99FF33",
    strokeWidth: 4
});

// A marker for move_base goals
var goalMarker = Kinetic.Circle();

// A line for setting goal poses
goalPoseLine = Kinetic.Line();

// A flag to indicate when the mouse is down
var mouseDown = false;

// Alert colors
var green = "#33FF33";
var yellow = "FFCC33";
var red = "FF1111";

function log(msg) {
    javascript:console.log(msg);
}

function getScreenWidth() {
    return screenWidth;
}

function getVideoWidth() {
    return videoWidth;
}

function getVideoHeight() {
    return videoHeight;
}

// Make the connection to the rosbridge server
log('Creating ROSProxy connection object...');
var connection = null;
try {
    connection = new ros.Bridge("ws://" + rosbridgeHost + ":" + rosbridgePort);
} catch (err) {
    log('Failed to connect to rosbridge!');
    return;
}

log("Connection created to " + rosbridgeHost + ":" + rosbridgePort);

// Create callbacks on the rosbridge open, error and close events
connection.onError(function() {
    log('rosbridge error!');
});

connection.onClose(function() {
    log('rosbridge closed');
});

connection.onOpen(function() {
/*
    log('Initializing ROSProxy...');
    try {
	connection.callService('/rosjs/topics', '[]', function nop() {});
	return;
    } catch (err) {
        log('Problem initializing ROSProxy!');
        return;
    }
*/
    // Call the special rosbridge service get_param to read in various parameters
    connection.callService('/rosbridge/get_param', '["pi_rosbridge"]', function(resp) {
	for (param in resp) {
	    log(param + ": " + resp[param]);
	}
    	if (resp['max_linear_speed'] != null) maxLinearSpeed = resp['max_linear_speed'];
    	if (resp['max_angular_speed'] != null) maxAngularSpeed = resp['max_angular_speed'];
    	if (resp['map_file'] != null) mapFile = resp['map_file'];
    	if (resp['turtlebot'] != null) turtleBot = resp['turtlebot'];

	// If this is a TurtleBot, get the batter charge from /turtlebot_node/sensor_state.
	// Throttle to 1 Hz.
	if (turtleBot) {
    	    connection.addHandler(turtleBotBatteryTopic, function(msg) {
		var color;
		var charge = msg.charge;
		var capacity = msg.capacity;
		var remaining = 100 * (1.0 - ((capacity - charge) / (capacity)));
		$('#robotBattery').jqxLinearGauge('value', remaining);
		if (remaining < 35) { color = red; }
		else if (remaining < 55) { color = yellow; }
		else { color = green; }
		$('#robotBattery').jqxLinearGauge({ pointer: { pointerType: 'default', size: 20, offset: 10, style: {fill: color}}});
	    });
	    connection.callService('/rosjs/subscribe', '["' + turtleBotBatteryTopic + '", 1000]', function(e) {
		$('#robotBattery').jqxLinearGauge({ background: { style: { stroke: '#cccccc', fill: '#cccccc'}, visible: true, backgroundType: 'round' }});
		log('Subscribed to ' +  turtleBotBatteryTopic);
	    });
	}
	else {
	    // Otherwise, assume the robot battery topic is /serializer/sensors (Pi Robot) and display the
	    // charge status. Throttle to 1 Hz.
	    connection.addHandler(robotBatteryTopic, function(msg) {
		var color;
		var voltage = msg.value[0];
		var remaining = 100 * (voltage - 12.0) / (14.2 - 12.0)
		$('#robotBattery').jqxLinearGauge('value', remaining);
		if (remaining < 35) { color = red; }
		else if (remaining < 55) { color = yellow; }
		else { color = green; }
		$('#robotBattery').jqxLinearGauge({ pointer: { pointerType: 'default', size: 20, offset: 10, style: {fill: color}}});
	    });
	    connection.callService('/rosjs/subscribe', '["' + robotBatteryTopic + '", 1000]', function(e) {
		$('#robotBattery').jqxLinearGauge({ background: { style: { stroke: '#cccccc', fill: '#cccccc'}, visible: true, backgroundType: 'round' }});
		log('Subscribed to ' +  robotBatteryTopic);
	    });
	}
    });
    
    // Subscribe to the /map_metadata topic to get map parameters
    connection.addHandler(mapMetadataTopic, function(msg) {
	mapResolution = msg.resolution;
	mapWidth = msg.width;
	mapHeight = msg.height;
	mapOriginX = msg.origin.position.x;
	mapOriginY = msg.origin.position.y;

	// Once a map is detected, set the drawing parameters for navigation
	var mapStage = new Kinetic.Stage({
	    container: "map_container",
	    draggable: false,
	    width: mapWidth * mapScale + 10,
	    height: mapHeight * mapScale + 10,
	    x: 0,
	    y: 0
	});

	var mapImage = new Image();
	var map = Kinetic.Image();
	mapImage.onload = function() {
	    map = new Kinetic.Image({
		image: mapImage,
		width: mapWidth * mapScale,
		height: mapHeight * mapScale,
		offset: [0, 0],
		x: 0,
		y: 0
	    });

	    goalMarker = new Kinetic.Circle({
		x: mapStage.getWidth() / 2,
		y: mapStage.getHeight() / 2,
		radius: 10,
		fill: "#00CC00",
		stroke: "black",
		strokeWidth: 1
	    });

	    robotMarker = new Kinetic.Circle({
		x: mapStage.getWidth() / 2,
		y: mapStage.getHeight() / 2,
		radius: 15,
		fill: "orange",
		stroke: "black",
		strokeWidth: 1
	    });

	    goalPoseLine = new Kinetic.Line({
		points: [0, 0, 0, 0],
		strokeWidth: 3,
		stroke: "#00CC00"
	    });

	    mapMarkerLayer.add(goalPoseLine);

	    moving = false;

	    mapStage.on("mousedown touchstart", function(){
		if (moving){
		    moving = false; mapMarkerLayer.draw();
		} else {
		    if (is_touch_device) var mousePos = mapStage.getTouchPosition();
		    else var mousePos = mapStage.getMousePosition();
		    mapMarkerLayer.clear();
		    if (is_touch_device) var mousePos = mapStage.getTouchPosition();
		    else var mousePos = mapStage.getMousePosition();
		    var x =  mousePos.x;
		    var y =  mousePos.y;
		    goalMarker.setX(x);
		    goalMarker.setY(y);
		    mapMarkerLayer.add(goalMarker);

		    // Start point and end point are the same
		    goalPoseLine.getPoints()[0].x = mousePos.x;
		    goalPoseLine.getPoints()[0].y = mousePos.y;
		    goalPoseLine.getPoints()[1].x = mousePos.x;
		    goalPoseLine.getPoints()[1].y = mousePos.y;

		    moving = true;    
		    mapMarkerLayer.drawScene();            
		}

	    });

	    mapStage.on("mousemove touchmove", function(){
		if (moving) {
		    if (is_touch_device) var mousePos = mapStage.getTouchPosition();
		    else var mousePos = mapStage.getMousePosition();
		    var x = mousePos.x;
		    var y = mousePos.y;
		    goalPoseLine.getPoints()[1].x = mousePos.x;
		    goalPoseLine.getPoints()[1].y = mousePos.y;
		    moving = true;
		    mapMarkerLayer.drawScene();
		}
	    });

	    mapStage.on("mouseup touchend", function(){
		moving = false; 
	    });

	    // Add the image to the map layer
	    mapLayer.add(map);
	    mapRobotLayer.add(robotMarker);

	    // Add the map layer to the map stage
	    mapStage.add(mapLayer);
	    mapStage.add(mapMessageLayer);
	    mapStage.add(mapMarkerLayer);
	    mapStage.add(mapRobotLayer);
	};

	// Connect to the map server to get the map URL
	mapImage.src = "http://" + thisHost + mapFile;

    });
    connection.callService('/rosjs/subscribe', '["' + mapMetadataTopic + '",-1]', function(e) {
        log('Subscribed to ' + mapMetadataTopic);
    });

    // Subscribe to the /odom topic to get the robot's position. Throttle to 10 Hz.
    connection.addHandler(odomTopic, function(msg) {
	odomPositionX = msg.pose.pose.position.x;
	odomPositionY = msg.pose.pose.position.y;
	var x =  (odomPositionX - mapOriginX) / mapResolution * mapScale;
	var y = (mapHeight - (odomPositionY - mapOriginY) / mapResolution) * mapScale;;
	if (! usingAMCL) {
	    try {
		robotMarker.setX(x);
		robotMarker.setY(y);
		mapRobotLayer.draw();
	    } catch (err) {
		return;
	    }
	}
    });

    connection.callService('/rosjs/subscribe', '["' + odomTopic + '", 100]', function(e) {
        log('Subscribed to ' + odomTopic);
    });

    // Subscribe to the /amcl_pose topic to get the robot's position during SLAM.
    // Throttle to 10 Hz.
    connection.addHandler(amclPoseTopic, function(msg) {
	usingAMCL = true;
	amclPositionX = msg.pose.pose.position.x;
	amclPositionY = msg.pose.pose.position.y;
	var x =  (amclPositionX - mapOriginX) / mapResolution * mapScale;
	var y = (mapHeight - (amclPositionY - mapOriginY) / mapResolution) * mapScale;;
	if (usingAMCL) {
	    try {
		robotMarker.setX(x);
		robotMarker.setY(y);
		mapRobotLayer.draw();
	    } catch (err) {
		return;
	    }
	}
    });

    connection.callService('/rosjs/subscribe', '["' + amclPoseTopic + '", 100]', function(e) {
        log('Subscribed to ' + amclPoseTopic);
    });

    // Subscribe to the /joint_states topic to monitor servos and other joints. Throttle to 5 Hz.
    /*
    connection.addHandler(jointStatesTopic, function(msg) {
	jointNames = msg.name;
	jointPos = msg.position;
	jointVel = msg.velocity;
    });

    connection.callService('/rosjs/subscribe', '["' + jointStatesTopic + '", 500]', function(e) {
        log('Subscribed to ' + jointStatesTopic);
    });
    */

    // Subscribe to the /laptop_battery topic and display the charge status.
    // Throttle to 1 Hz.
    connection.addHandler(laptopBatteryTopic, function(msg) {
	var remaining = msg.percentage;
	var charging = msg.charge_state;
        var color;
        $('#laptopBattery').jqxLinearGauge('value', remaining);
        if (remaining < 35) { color = red; }
        else if (remaining < 55) { color = yellow; }
        else { color = green; }
	$('#laptopBattery').jqxLinearGauge({ pointer: { pointerType: 'default', size: 20, offset: 10, style: {fill: color}}});
	var laptop_battery_status = document.getElementById("laptop_battery_status");
	if (charging) {
	    laptop_battery_status.innerHTML = '<img src="images/battery_charging.png" align="absmiddle" width="56px" height="56px"> Laptop Charging ';
	    laptop_battery_status.style.fontSize = '18px';
	}
	else {
	    laptop_battery_status.style.fontSize = '18px';
	    laptop_battery_status.innerHTML = 'Laptop Battery';
	}
    });
    connection.callService('/rosjs/subscribe', '["' + laptopBatteryTopic + '", 1000]', function(e) {
        log('Subscribed to ' +  laptopBatteryTopic);
    });

    // Subscribe to the head pan state topic to get servo temperature. Throttle to 1 Hz.
    connection.addHandler(headPanTempTopic, function(msg) {
	var temp = msg.motor_temps[0];
        var color;
        $('#headPanTemp').jqxLinearGauge('value', temp);
        if (temp > 50) { color = red; }
        else if (temp > 45) { color = yellow; }
        else { color = green; }
	$('#headPanTemp').jqxLinearGauge({pointer: {pointerType: 'default', size: 30, offset: 0, style: {fill: color}}});
	$('#servoStatus').jqxButton({style: {fill: color}});
    });
    connection.callService('/rosjs/subscribe', '["' + headPanTempTopic + '", 1000]', function(e) {
        log('Subscribed to ' +  headPanTempTopic);
    });

    // Subscribe to the head tilt state topic to get servo temperature.  Throttle to 1 Hz.
    connection.addHandler(headTiltTempTopic, function(msg) {
	var temp = msg.motor_temps[0];
        var color;
        $('#headTiltTemp').jqxLinearGauge('value', temp);
        if (temp > 50) { color = red; }
        else if (temp > 45) { color = yellow; }
        else { color = green; }
	$('#headTiltTemp').jqxLinearGauge({pointer: {pointerType: 'default', size: 30, offset: 0, style: {fill: color}}});
	$('#servoStatus').jqxButton({style: {fill: color}});
    });
    connection.callService('/rosjs/subscribe', '["' + headTiltTempTopic + '", 1000]', function(e) {
        log('Subscribed to ' +  headTiltTempTopic);
    });

    // Subscribe to the /roi topic to get the region of interest.  Throttle to 10 Hz.
    connection.addHandler(roiTopic, function(msg) {
	roiOffsetX = msg.x_offset;
	roiOffsetY = msg.y_offset;
	roiWidth = msg.width;
	roiHeight = msg.height;
	roiMarker.setX(roiOffsetX);
	roiMarker.setY(roiOffsetY);
	roiMarker.setWidth(roiWidth);
	roiMarker.setHeight(roiHeight);
	videoMarkerLayer.draw();
    });
    connection.callService('/rosjs/subscribe', '["' + roiTopic + '", 100]', function(e) {
	log('Subscribed to ' + roiTopic);
    });

    // Subscribe to the /obstacle_laser topic to determine if there is an obstacle ahead
    connection.addHandler(obstacleLaserTopic, function(msg) {
	obstacleDetected = msg.data;
/*
	if (obstacleDetected) {
	    writeMessage(baseMessageLayer, "Obstacle Detected!");
	}
*/
    });
    connection.callService('/rosjs/subscribe', '["' + obstacleLaserTopic + '", -1]', function(e) {
	log('Subscribed to ' + obstacleLaserTopic);
    });

    // Subscribe to the base laser /scan topic. Throttle to 5Hz
/*
    connection.addHandler(baseLaserScanTopic, function(msg) {
	var nranges = msg.ranges.length;
	baseLaserScanArray = new Array();
	for (i = 0; i < nranges; i++) {
	    row = {};
	    row['index'] = i;
	    row['range'] = msg.ranges[i] * 1;
	    baseLaserScanArray[i] = row;
	}
	//$('#baseLaserChart').jqxChart({source: baseLaserScanArray});
	//$('#baseLaserChart').jqxChart('refresh');
    });

    connection.callService('/rosjs/subscribe', '["' + baseLaserScanTopic + '", 200]', function(e) {
        log('Subscribed to ' + baseLaserScanTopic);
    });
*/

    connection.callService(headPanSpeedService, '{"speed":' + defaultHeadPanSpeed + '}', function() {}); 
    connection.callService(headTiltSpeedService, '{"speed":' + defaultHeadTiltSpeed + '}', function() {}); 

    // Set failsafe max linear and angular speeds in case the parameters are not picked up
    if (maxLinearSpeed == null) {
	maxLinearSpeed = defaultMaxLinearSpeed;
	$('#baseLinearSpeedSlider').jqxSlider('value', maxLinearSpeed);
    }

    if (maxAngularSpeed == null) {
	maxAngularSpeed = defaultMaxAngularSpeed;
	$('#baseAngularSpeedSlider').jqxSlider('value', maxAngularSpeed);
    }	

    function handleKey(code, down) {
	if (!down) return;

        switch (code) {
        case 32:
            // Space bar
	    vx = 0;
	    vz = 0;
            break;
        case 37:
            // Left arrow
	    if (vz < 0) vz = 0;
            vz += vz_key_increment;
	    vx += -sign(vx) * vx_key_increment;
            break;
        case 38:
            // Up arrow
	    vz = 0;
            vx += vx_key_increment;
            break;
        case 39:
            // Right arrow
	    if (vz > 0) vz = 0;
            vz -= vz_key_increment;
	    vx += -sign(vx) * vx_key_increment;
            break;
        case 40:
            // Down arrow
	    vz = 0;
            vx -= vx_key_increment;
            break;
        }
	pubCmdVel(vx, vz);
    }

    document.addEventListener('keydown', function (e) {
        handleKey(e.keyCode, true);
    }, true);

    document.addEventListener('keyup', function (e) {
        handleKey(e.keyCode, false);
    }, true);

    // Start the video and publisher loops
    log("HELLO!");
    videoOn();
    pubLoop();
});

function twistMsg(x, z) {
   return '{"linear":{"x":' + x + ',"y":0,"z":0},"angular":{"x":0,"y":0,"z":' + z + '}}';
}

function pubCmdVel(_vx, _vz) {
    // If an obstacle is detected in front, prevent forward motion
    if (obstacleDetected && _vx > 0) {
	_vx = 0;
    }
    vx = sign(_vx) * Math.min(maxLinearSpeed, Math.abs(_vx))
    vz = sign(_vz) * Math.min(maxAngularSpeed, Math.abs(_vz))
    connection.publish(cmdVelTopic, 'geometry_msgs/Twist', twistMsg(vx, vz));
}

function moveBaseMsg(x, y, qz, qw) {
    return '{"header":{"frame_id":"/map"},"pose":{"position":{"x":' + x + ',"y":' + y + ',"z":0},"orientation":{"x":0,"y":0,"z":' + qz + ',"w":' + qw + '}}}';
}

function poseMsg(x, y, qz, qw) {
    // NOTE: The time stamp *must* be set to 0,0 for this to work.
    return '{"header":{"frame_id":"/map","stamp":{"secs":0, "nsecs":0}},"pose":{"pose":{"position":{"x":' + x + ',"y":' + y + ',"z":0},"orientation":{"x":0,"y":0,"z":' + qz +',"w":' + qw + '}}}}';
}

function setPose() {
    var x =  goalMarker.getX() * mapResolution / mapScale + mapOriginX;
    var y =  (mapHeight * mapScale - goalMarker.getY()) * mapResolution / mapScale + mapOriginY;
    var dx = goalPoseLine.getPoints()[1].x - goalPoseLine.getPoints()[0].x;
    var dy = goalPoseLine.getPoints()[1].y - goalPoseLine.getPoints()[0].y;
    var theta = -Math.atan2(dy, dx);
    var qz = Math.sin(theta / 2);
    var qw = Math.cos(theta / 2);
    connection.publish(initialPoseTopic, 'geometry_msgs/PoseWithCovarianceStamped', poseMsg(x, y, qz, qw));
    connection.publish(amclPoseTopic, 'geometry_msgs/PoseWithCovarianceStamped', poseMsg(x, y, qz, qw));

    // Not sure why, but publishing has to be done twice--but on the first time!
    if (firstSetPose) {
	sleep(500);
	connection.publish(initialPoseTopic, 'geometry_msgs/PoseWithCovarianceStamped', poseMsg(x, y, qz, qw));
	connection.publish(amclPoseTopic, 'geometry_msgs/PoseWithCovarianceStamped', poseMsg(x, y, qz, qw));
	firstSetPose = false;
    }
}

function moveBase() {
    var x =  goalMarker.getX() * mapResolution / mapScale + mapOriginX;
    var y =  (mapHeight * mapScale - goalMarker.getY()) * mapResolution / mapScale + mapOriginY;
    var dx = goalPoseLine.getPoints()[1].x - goalPoseLine.getPoints()[0].x;
    var dy = goalPoseLine.getPoints()[1].y - goalPoseLine.getPoints()[0].y;
    var theta = -Math.atan2(dy, dx);
    var qz = Math.sin(theta / 2);
    var qw = Math.cos(theta / 2);
    connection.publish(moveBaseTopic, 'geometry_msgs/PoseStamped', moveBaseMsg(x, y, qz, qw));

    // Not sure why, but publishing has to be done twice--but on the first time!
    if (firstMoveBase) {
	sleep(500);
	connection.publish(moveBaseTopic, 'geometry_msgs/PoseStamped', moveBaseMsg(x, y, qz, qw));
	firstMoveBase = false;
    }
}

function moveBaseCancel() {
    connection.publish('/move_base/cancel', 'actionlib_msgs/GoalID', '{"id":"0"}');

    // Not sure why, but publishing has to be done twice--but on the first time!
    if (firstMoveBaseCancel) {
	sleep(50);
	connection.publish('/move_base/cancel', 'actionlib_msgs/GoalID', '{"id":"0"}');
	firstMoveBaseCancel = false;
    }
}

function pubHeadCmd() {
    if (centerServos){
        servosRelaxed = false;
	centerServos = false;
        connection.publish(headPanPositionTopic, 'std_msgs/Float64', '{"data": 0}');
        connection.publish(headTiltPositionTopic, 'std_msgs/Float64', '{"data": 0}');
	
    }
    else if (lockServos){
        servosRelaxed = false;
		centerServos = false;
		lockServos = false;
        connection.publish(headPanPositionTopic, 'std_msgs/Float64', '{"data": ' + jointPos[0] + '}');
        connection.publish(headTiltPositionTopic, 'std_msgs/Float64', '{"data": ' + jointPos[1] + '}');
	
    }
    else if (relaxServos && !servosRelaxed) {
        servosRelaxed = true;
        connection.callService(headPanTorqueService, '{"torque_enable": false}', function() {}); 
        connection.callService(headTiltTorqueService, '{"torque_enable": false}', function() {}); 
    }
    else {
        servosRelaxed = false;
        connection.publish(headPanPositionTopic, 'std_msgs/Float64', '{"data":' + headPanPosition + '}');
        connection.publish(headTiltPositionTopic, 'std_msgs/Float64', '{"data":' + headTiltPosition + '}');
    }
}

function relaxAllServos() {
    relaxServos = true;
    servosRelaxed = false;
    writeMessage(servoMessageLayer, "Servos relaxed");
    pubHeadCmd();
}

function centerHeadServos() {
    writeMessage(servoMessageLayer, "Centering servos");
    servoPadMarker.setX(servoPadWidth/2);
    servoPadMarker.setY(servoPadHeight/2);
    servoMarkerLayer.draw();
    centerServos = true;
    lockServos = false;
    headPanSpeed = defaultHeadPanSpeed;
    headTiltSpeed = defaultHeadTiltSpeed;
    headTiltPosition = headPanPosition = 0;
    pubHeadCmd();
}

function stopRobot() {
    pubCmdVel(0, 0);
    writeMessage(baseMessageLayer, "Stopping robot");
}

function refreshVideo() {
    log("Frame");
    videoStage.draw();
//    videoLayer.draw();
}

function refreshPublishers() {
    if (mouseDown) {
	pubCmdVel(vx, vz);
    }
}

function pubLoop() {
    pubHandle = setInterval(refreshPublishers, 1000 / rate);
}

function videoOn() {
    <!-- Redraw of video canvas every 1000 / videoFPS ms -->
    videoHandle = setInterval(refreshVideo, 1000 / videoFPS);
    video_on.style.background = '#55bb55';
    video_on.style.color = 'white';
    video_off.style.background = '#DDDDDD';
    video_off.style.color = 'black';
}

function videoOff() {
    clearInterval(videoHandle);
    video_off.style.background = '#f24537';
    video_off.style.color = 'white';
    video_on.style.background = '#DDDDDD';
    video_on.style.color = 'black';
}

function sleep(delay) {
    var start = new Date().getTime();
    while (new Date().getTime() < start + delay);
}

function sign(x)
{
    if (x < 0) {return -1};
    if (x > 0) {return 1};
    return 0;
}

// Begin KineticJS scripts and parameters
var basePadWidth = 250;
var basePadHeight = 250;
var servoPadWidth = 250;
var servoPadHeight = 250;
/*
var mapWidth = 237;
var mapHeight = 233;
var mapResolution = 0.05;
var mapOriginX = -6.0;
var mapOriginY = -5.0;
*/
var mapScale = 2.0;

function writeMessage(messageLayer, message) {
    var context = messageLayer.getContext();
    messageLayer.clear();
    context.font = "18pt Calibri";
    context.fillStyle = "black";
    context.fillText(message, 10, 25);
}

var videoStage = new Kinetic.Stage({
    container: "video_container",
    draggable: false,
    width: videoWidth * videoScale,
    height: videoHeight * videoScale,
    x: 0,
    y: 0
});

var baseStage = new Kinetic.Stage({
    container: "base_container",
    draggable: false,
    width: basePadWidth,
    height: basePadHeight,
    x: 0,
    y: 0
});

var servoStage = new Kinetic.Stage({
    container: "servo_container",
    draggable: false,
    width: servoPadWidth,
    height: servoPadHeight,
    x: 0,
    y: 0
});

var videoLayer = new Kinetic.Layer();
var videoMarkerLayer = new Kinetic.Layer();
var videoMessageLayer = new Kinetic.Layer();
var baseLayer = new Kinetic.Layer();
var baseMarkerLayer = new Kinetic.Layer();
var baseMessageLayer = new Kinetic.Layer();
var servoLayer = new Kinetic.Layer();
var servoMarkerLayer = new Kinetic.Layer();
var servoMessageLayer = new Kinetic.Layer();
var mapLayer = new Kinetic.Layer();
var mapMarkerLayer = new Kinetic.Layer();
var mapMessageLayer = new Kinetic.Layer();
var mapRobotLayer = new Kinetic.Layer();

<!-- The Video Panel -->
var videoImage = new Image();
var video = Kinetic.Image();
videoImage.onload = function() {
    video = new Kinetic.Image({
	image: videoImage,
	width: videoWidth * videoScale,
	height: videoHeight * videoScale,
	offset: [0, 0],
	x: 0,
	y: 0
    });

    videoLayer.add(video);
    videoMarkerLayer.add(roiMarker);
    videoStage.add(videoLayer);
    videoStage.add(videoMarkerLayer);
}

// Connect to the mjpeg server for streaming video
videoImage.src = "http://" + rosbridgeHost + ":" + mjpegPort + "/stream?topic=" + videoTopic;

<!-- The BasePad -->
var basePad = new Kinetic.Rect ({
    x: baseStage.getWidth() - basePadWidth,
    y: baseStage.getHeight() - basePadHeight,
    width: basePadWidth,
    height: basePadHeight,
    offset: [0, 0],
    fill: "#00D2FF",
    stroke: "black",
    strokeWidth: 4
});

var basePadMarker = new Kinetic.Circle({
    x: basePadWidth/2,
    y: basePadHeight/2,
    radius: 25,
    listening: false,
    fill: "yellow",
    stroke: "black",
    strokeWidth: 1
});

basePad.on("mousedown touchstart", function() {
    mouseDown = true;
});

basePad.on("mousemove touchmove", function() {
    if (! is_touch_device && ! mouseDown) { return; }
    if (is_touch_device) var mousePos = baseStage.getTouchPosition();
    else var mousePos = baseStage.getMousePosition();
    basePadMarker.setX(mousePos.x);
    basePadMarker.setY(mousePos.y);
    baseMarkerLayer.draw();
    var x = (mousePos.x - basePad.getX()) - basePadWidth / 2;
    var y = basePadHeight / 2 - (mousePos.y - basePad.getY());
    x /= basePadWidth / 2;
    y /= basePadHeight / 2;
    vx = sign(y) * (Math.pow(2, Math.abs(y)) - 1) * maxLinearSpeed;
    vz = -sign(x) * (Math.pow(2, Math.abs(x)) - 1) * maxAngularSpeed;
    writeMessage(baseMessageLayer, " vx: " + Math.round(x * 100)/100 + ", vz: " + Math.round(y*100)/100);
    pubCmdVel(vx, vz);
});

basePad.on("touchend mouseup dblclick", function() {
    mouseDown = false;
    pubCmdVel(0, 0);
    basePadMarker.setX(basePadWidth/2);
    basePadMarker.setY(basePadHeight/2);
    baseMarkerLayer.draw();
    writeMessage(baseMessageLayer, "Stopping robot");
});

baseLayer.add(basePad);
baseMarkerLayer.add(basePadMarker);

<!-- The Servo Pad -->
var servoPad = new Kinetic.Rect ({
    x: servoStage.getWidth() - servoPadWidth,
    y: servoStage.getHeight() - servoPadHeight,
    width: servoPadWidth,
    height: servoPadHeight,
    offset: [0, 0],
    fill: "#00FF99",
    stroke: "black",
    strokeWidth: 4
});

var servoPadMarker = new Kinetic.Circle({
    x: servoPadWidth/2,
    y: servoPadHeight/2,
    radius: 25,
    listening: false,
    fill: "yellow",
    stroke: "black",
    strokeWidth: 1
});

servoPad.on("mousedown touchstart", function() {
    mouseDown = true;
});


servoPad.on("touchend touchleave touchcancel mouseup mouseout", function() {
    mouseDown = false;
/*
    if (servosRelaxed) {
	relaxServos = true;
	lockServos = false;
	writeMessage(servoMessageLayer, "Servos relaxed");
    }
    else {
	relaxServos = false;
	lockServos = true;
	writeMessage(servoMessageLayer, "Locking servos");
    }
    pubHeadCmd();
*/
});


servoPad.on("mousemove touchmove", function() {
    relaxServos = false;
    lockServos = false;
    if (! is_touch_device && ! mouseDown) { return; }
    if (is_touch_device) var mousePos = servoStage.getTouchPosition();
    else var mousePos = servoStage.getMousePosition();
    servoPadMarker.setX(mousePos.x);
    servoPadMarker.setY(mousePos.y);
    servoMarkerLayer.draw();
    var x = (mousePos.x - servoPad.getX()) - servoPadWidth / 2;
    var y = servoPadHeight / 2 - (mousePos.y - servoPad.getY());
    x /= servoPadWidth / 2;
    y /= servoPadHeight / 2;
    writeMessage(servoMessageLayer, " pan: " + Math.round(x * 100)/100 + ", tilt: " + Math.round(y*100)/100);

    if (servoSpeedControl) {
	headTiltSpeed = Math.abs(y * maxHeadTiltSpeed);
	headPanSpeed = Math.abs(x * maxHeadPanSpeed);

	if (y >= 0) headTiltPosition = maxHeadTiltPosition;
	else headTiltPosition = -maxHeadTiltPosition;

	if (x >= 0) headPanPosition = -maxHeadPanPosition;
	else headPanPosition = maxHeadPanPosition;
    }
    else {
	headPanSpeed = defaultHeadPanSpeed;
	headTiltSpeed = defaultHeadTiltSpeed;
	headPanPosition = -x * maxHeadPanPosition;
	headTiltPosition = -y * maxHeadTiltPosition;
	headPanPosition = sign(headPanPosition) * Math.min(maxHeadPanPosition, Math.abs(headPanPosition));
	headTiltPosition = sign(headTiltPosition) * Math.min(maxHeadTiltPosition, Math.abs(headTiltPosition));
    }
    pubHeadCmd();
});

servoPad.on("dbltap dblclick", function() {
    centerHeadServos();
});

servoLayer.add(servoPad);
servoMarkerLayer.add(servoPadMarker);

function zoomInMap() {
    mapStage.setWidth(mapStage.getWidth() * 1.1);
    mapStage.setHeight(mapStage.getHeight() * 1.1);
    map.setWidth(map.getWidth() * 1.1);
    map.setHeight(map.getHeight() * 1.1);
    mapLayer.draw();
}

function zoomOutMap() {
    mapStage.setWidth(mapStage.getWidth() / 1.1);
    mapStage.setHeight(mapStage.getHeight() / 1.1);
    map.setWidth(map.getWidth() / 1.1);
    map.setHeight(map.getHeight() / 1.1);
    mapLayer.draw();
}

// Add other layers to stages
baseStage.add(baseLayer);
baseStage.add(baseMarkerLayer);
baseStage.add(baseMessageLayer);

servoStage.add(servoLayer);
servoStage.add(servoMarkerLayer);
servoStage.add(servoMessageLayer);

mapLayer.draw();
mapRobotLayer.draw();




