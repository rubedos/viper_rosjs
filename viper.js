/**
 * @author Henrikas Simonavičius / henrikas.simonavicius@rubedos.com
 * @author Rubedos / http://www.rubedos.com
 */
//import { OrbitControls } from '3rdparty/controls/OrbitControls.js';

var __instance = null;
var cvmAPIservice = "/configurator/cvmapi";
var listeners = new Map();
/** 
 */
function VIPER(options) {
	__instance = this;
	this.ros_ip = options.viper_ip;
	this.connected = false;
	this.ros = null;
	this.isSimulation = (this.ros_ip == 'localhost');	// For debugging purposes
	
	if (this.isSimulation) console.log('Running in simulation mode. Some services may be not available');


/** Connects to viper device
 */
this.connect = function() {
  if (!this.connected)
  {
    // Connect to ROS
    this.ros = new ROSLIB.Ros({
      url : 'ws://' + this.ros_ip + ':9090'
    });

    this.ros.on('connection', function(){
      __instance.log('Connected to websocket server @ ' + __instance.ros.socket.url);
	  VIPER.getDeviceInfo(__instance);
	  VIPER.getTopics(__instance);
	  VIPER.checkStatus(__instance);
	});

    this.ros.on('error', function(error) {
      __instance.log('Error connecting to websocket server: ', error);
      VIPER.checkStatus(__instance);
	  alert('Error connecting to websocket server: ' + error);
    });

    this.ros.on('close', function() {
      __instance.log('Connection to websocket server closed.');
      VIPER.checkStatus(__instance);
	  alert('Connection to websocket server closed.');
    });
  }
  else
  {
    VIPER.checkStatus(this);
	this.close();
  }

}

this.close = function() {
	this.connected = false;
    if (this.ros != null)
    {
	  //this.ros.socket.close();
      this.ros.close();
      this.ros = null;
    }
	this.log('Closing connection to ' + this.ros_ip);
}

/** Calls a service to activate/deactivate specified VIPER app
 */
this.activateApp = function(appId, activate, extRosMaster) {
	var prm1 = appId;
	var prm2 = '';
	var prm3 = '';
	if (extRosMaster == true) prm3 = 'yes';
	var cmd = 'activateapp';
	if (!activate) cmd = 'deactivateapp';
	if (appId == 'cvm-follow-me') prm2 = 'roslaunch cvm_follow_me follow_me.launch';
	if (appId == 'cvm-stereo') prm2 = 'roslaunch cvm_v4l2_camera v4l2_stereo.launch';
	if (appId == 'cvm-obstacle-detector') prm2 = 'roslaunch cvm_laserscan laserscan.launch';
	if (appId == 'cvm-follow-aruco') prm2 = 'roslaunch cvm_follow_aruco follow_aruco.launch';
	
	var request = new ROSLIB.ServiceRequest({
	'auth' : 'labas', 'cmd' : cmd, 'prm1' : prm1, 'prm2' : prm2, 'prm3' : prm3
	});
	
	var cvmAPI = this.getCvmService();
	
	cvmAPI.callService(request, function(result) {
		viper.log('CVM API service responded to request "' + request.cmd + '". RC: '+result.rc);
	});
}

/** Call this method to reset imu orientation fusion (i.e. current VIPER orientation becomes 0, 0 ,0).
	@param finishedCallback is invoked when reset has been finished.
 */
this.resetImu = function(finishedCallback) {
	var cmd = 'resetimu';
	
	var request = new ROSLIB.ServiceRequest({
		'auth' : 'labas', 'cmd' : cmd, 'prm1' : '', 'prm2' : '', 'prm3' : ''
	});
	this.getCvmService().callService(request, function(result) {
		viper.log('CVM API service responded to request "' + request.cmd + '". RC: '+result.rc);
		setTimeout(function(){ finishedCallback(); }, 1000);
	});
	
}

this.getTopicType = function(topic) {
	if (this.topics == null) throw "Topic list is empty";
	var type = null;
	if (topic[0] != '/') topic = '/' + topic;
	for (var [ttopic, ttype] of __instance.topics)
		if (ttopic == topic) type = ttype;
	if (type == null) throw 'Topic ' + topic + ' type is unknown';
	return type;
}

/** Publishes message to specified topic. 
	@param topic - the name of the topic
	@param data - topic messate data part.
 */
this.publishTopic = function(topic, msg, latched) {
	this.log("Publishing to topic '" + topic + "'");
	var type = this.getTopicType(topic)
	var topicHandle = new ROSLIB.Topic({
		ros: this.ros,
		name: topic, 
		messageType: type,
		latched: latched
	});
	/*var msgTransport = new ROSLIB.Message({
		data: data
	}); */
	topicHandle.publish(msg);
}

this.subscribeTopic = function(topic, callback) {
	__instance.log('Subscribing to topic ', topic);
	var type = this.getTopicType(topic)
	var listener = new ROSLIB.Topic({
		ros : __instance.ros,
		name : topic,
		messageType : type,
		queue_size : 1
	});
	listener.subscribe(function(message) {
		callback(message);
	});
	
	return listener;
}

/** Subscribe to image topic and sends updated image data to given callback.
 Function returns listener which is a handle to call unsubscribe when finished streaming.
 @param topic - the name of image topic
 @param callback(rgbByteBuffer, width, height) - a function to be called with updated image data (RGB byte buffer, image width, image height).
 @return subscription listener
 */
this.subscribeImage = function(topic, callback){
	__instance.log('Subscribing to Image topic ', topic);
	var type = this.getTopicType(topic)
	var listener = new ROSLIB.Topic({
		ros : __instance.ros,
		name : topic,
		messageType : type,
		queue_size : 1
	});
	//listeners.set(listener.name, callback);
	listener.subscribe(function(message) {
		var w = message.width;
		var h = message.height;
		var buffer = new Uint8Array(Base64Binary.decodeArrayBuffer(message.data));
		callback(buffer, w, h);
  });

  return listener;
}

/** Subscribes to cvm_msgs/Stereoimage topic and receives rgb and disparity buffers (base64). Decodes them to javascript arrays.
 @param topic - the name of steroimage topic
 @param callback(data) - a function to be called with updated image data (RGB byte buffer, Disparity buffer, image width, image height, focal point, baseline etc.).
 */
this.subscribeStereoimage = function(topic, callback){
	__instance.log('Subscribing to Stereoimage topic ', topic);
	var type = this.getTopicType(topic)
	var listener = new ROSLIB.Topic({
		ros : __instance.ros,
		name : topic,
		messageType : type,
		queue_size : 1
	});
	//listeners.set(listener.name, callback);
	listener.subscribe(function(message) {
		var w = message.leftImage.width;
		var h = message.leftImage.height;
		var rgbBuffer = new Uint8Array(Base64Binary.decodeArrayBuffer(message.leftImage.data));
		
		var focal = message.disparityImage.f;
		var baseline = message.disparityImage.T;
		var disparityBuffer = new DataView(Base64Binary.decodeArrayBuffer(message.disparityImage.image.data));
		callback({rgb:rgbBuffer, 
			disparity: disparityBuffer, 
			width: w, height: h, 
			focal: focal, baseline: baseline, 
			dispMin: message.disparityImage.min_disparity, 
			dispMax: message.disparityImage.max_disparity,
			frameId: message.header.frame_id});
  });

  return listener;
}

/** Subscribe to points topic and sends updated points data to given callback.
 Function returns listener which is a handle to call unsubscribe when finished streaming.
 @param topic - the name of points topic
 @param callback(pointsBuffer, width, height) - a function to be called with updated image data (http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html buffer as DataView, point array width, point array height).
 @return subscription listener
 */
this.subscribePoints = function(topic, callback) {
	__instance.log('Subscribing to Points topic ', topic);
	var type = this.getTopicType(topic)
	var listener = new ROSLIB.Topic({
		ros : __instance.ros,
		name : topic,
		messageType : type,
		queue_size : 1
	});
	listener.subscribe(function(message) {
		var w = message.width;
		var h = message.height;
		var buffer = new DataView(Base64Binary.decodeArrayBuffer(message.data));
		callback(buffer, w, h);
  });
  return listener;
}
/** Creates THREE JS buffer structure to store pointcloud model for rendering.
 */
this.createPointCloud3D = function(width, height) {
	var numPoints = width * height;

	var positions = new Float32Array( numPoints * 3 );
	var scales = new Float32Array( numPoints );
	var colors = new Float32Array( numPoints * 3 );

	var i = 0, j = 0;

	for ( var iy = 0; iy < height; iy ++ ) {
		for ( var ix = 0; ix < width; ix ++ ) {

			positions[ i ] = 0;
			positions[ i + 1 ] = 0; 
			positions[ i + 2 ] = 0;

			scales[ j ] = 0.01;
			colors[i] = 0;
			colors[i + 1] = 0;
			colors[i + 2] = 0;

			i += 3;
			j ++;
		}
	}

	var geometry = new THREE.BufferGeometry();
	geometry.addAttribute( 'position', new THREE.BufferAttribute( positions, 3 ) );
	geometry.addAttribute( 'color', new THREE.BufferAttribute( colors, 3 ) );

	var material = new THREE.ShaderMaterial( {
		uniforms: {
			//color: { value: new THREE.Color( 0xffffff ) },
			pointSize: {type: 'float', value: 5.0 }
		},
		vertexShader: VIPER.pointCloudVertexShader()
		,vertexColors: true
		,fragmentShader: VIPER.pointCloudFragmentShader()

	} );

	//

	points = new THREE.Points( geometry, material );
	return points;
}

/** Renders received image buffer onto canvas element
 */
this.drawImage = function(canvas, rgbBuffer, width, height) {
	canvas.width = width;
	canvas.height = height;
	var ctx = canvas.getContext('2d');
	var pixels = ctx.getImageData(0,0, width, height);
	var r, g, b;
	for (var y = 0; y < height; y++)
		for (var x = 0; x < width; x++)
		{
			var i = (y * width + x) * 3;  // 3 bytes per pixel 
			var pi = (y * width + x) * 4; // 4 bytes per pixel 
			r = rgbBuffer[i];
			g = rgbBuffer[i+1];
			b = rgbBuffer[i+2];
			pixels.data[pi] = r;
			pixels.data[pi+1] = g;
			pixels.data[pi+2] = b;
			pixels.data[pi+3] = 255;	// alpha
		}
	ctx.putImageData(pixels, 0, 0);
}

/** Renders received disparity buffer onto canvas element.
	@param canvas - html canvas element where to render disparity
	@param data - stereoimage containing decoded disparity buffer and more 
 */
this.drawDisparity = function(canvas, data) {
	canvas.width = data.width;
	canvas.height = data.height;
	var ctx = canvas.getContext('2d');
	var pixels = ctx.getImageData(0,0, data.width, data.height);
	for (var y = 0; y < data.height; y++)
		for (var x = 0; x < data.width; x++)
		{
			var i = (y * data.width + x) * 4;  // 1 float (4 bytes) per pixel 
			var pi = (y * data.width + x) * 4; // 4 bytes per pixel 
			var disp = data.disparity.getFloat32(i, true);
			if (disp > 0)
			{
				r = data.rgb[i];
				g = data.rgb[i+1];
				b = data.rgb[i+2];
				var val = (disp/(data.dispMax - data.dispMin) - data.dispMin)*255;
				pixels.data[pi] = val;
				pixels.data[pi+1] = val;
				pixels.data[pi+2] = val;
				pixels.data[pi+3] = 255;	// alpha
			}
			else
			{
				pixels.data[pi] = 100;
				pixels.data[pi+1] = 0;
				pixels.data[pi+2] = 0;
				pixels.data[pi+3] = 255;	// alpha
			}
		}
	ctx.putImageData(pixels, 0, 0);
}

/** Converts RGB + D maps to pointcloud for rendering
 @param data - decoded message received from VIPER containing RGB and Disparity buffers
 @param points3DBuffer - structure for pointcloud rendering 
 */
this.rgbdToCloud = function(data, points3DBuffer, pointSize) {
	var positions = points3DBuffer.geometry.attributes.position.array;
	var colors = points3DBuffer.geometry.attributes.color.array;
	// NOTE: principal point should come from calibrated camera info
	var principalX = data.width /2;
	var principalY = data.height /2;
	var invalidPoints = 0; var invalidVal = 0;
	for (var y = 0; y < data.height; y++)
		for (var x = 0; x < data.width; x++)
		{
			var ip = (y * data.width + x)*3;
			var i = (y * data.width + x) * 4;  // 1 float (4 bytes) per pixel 
			var disp = data.disparity.getFloat32(i, true);
			if (disp > 0)
			{
				cz = data.focal * data.baseline / (disp);
				var u = principalX - x;
				var v = principalY - y;
				var cx = u * cz / data.focal;
				var cy = v * cz / data.focal;
				r = data.rgb[ip];
				g = data.rgb[ip + 1];
				b = data.rgb[ip + 2];
				// Convert to ROS
				positions[ip] = cz; 
				positions[ip + 1] = cx; 
				positions[ip + 2] = cy; 
				colors[ip] = r/255; 
				colors[ip + 2] = b/255; 
				colors[ip + 1] = g/255; 
			}
			else
			{
				positions[ip] = invalidVal; 
				positions[ip + 1] = invalidVal; 
				positions[ip + 2] = invalidVal; 
				invalidPoints++;
			}
		}
	//this.log('Total points: ', data.height * data.width, ', invalids: ', invalidPoints);
	points3DBuffer.geometry.attributes.position.needsUpdate = true;
	points3DBuffer.geometry.attributes.color.needsUpdate = true;
	if (pointSize != null) {
		points3DBuffer.material.uniforms['pointSize'].value = pointSize;
		points3DBuffer.material.needsUpdate = true;
	}
}

/** This function uses ros PointCloud2 buffer data to update 3d model of the pointcloud
 @param rosBuffer - this parameter contains PointCloud2 structure. In case of VIPER it's a buffer of bytes where each pointcloud point
 has 32 bytes :
 offset		length		type		data
 0			4			float32		x coordinate
 4			4			float32		y coordinate
 8			4			float32		z coordinate
 12			4			float32		<reserved>
 16			1			byte		Red component
 17			1			byte		Green component
 18			1			byte		Blue component
 19			1			byte		<reserved>
 20			12			float32		3x floats, <reserved>
 
 
 NOTE: RGB is encoded as BGR, don't forget to swap R and B.
 XYZ is in ROS coordinate system.
*/
this.updatePointsBuffer = function(rosBuffer, points3DBuffer, w, h, pointSize) {
	var positions = points3DBuffer.geometry.attributes.position.array;
	var colors = points3DBuffer.geometry.attributes.color.array;
	var point_step = 32; 				// Received from PointCloud2 structure
	var row_step = w * point_step; 	// Received from PointCloud2 structure
	for (var j = 0; j < h; j++)
		for (var i = 0; i < w; i ++)
		{
			var ip = (j * w + i)*3;
			var ix = j * row_step + i*point_step;
			var x = rosBuffer.getFloat32(ix, true);
			var y = rosBuffer.getFloat32(ix + 4, true);
			var z = rosBuffer.getFloat32(ix + 8, true);
			var r = rosBuffer.getUint8(ix + 16);
			var g = rosBuffer.getUint8(ix + 17);
			var b = rosBuffer.getUint8(ix + 18);
			if (isFinite(x) && x > 1 && !isNaN(x) && ! isNaN(y) && !isNaN(z))
			{
				if (x > 20) x = 20;
				positions[ip] = x; 
				positions[ip + 1] = y; 
				positions[ip + 2] = z; 
				// NOTE: BGR to RGB transform
				colors[ip + 2] = r/255; 
				colors[ip + 1] = g/255; 
				colors[ip] = b/255; 
			}
			else 
			{
				positions[ip] = 0; 
				positions[ip + 1] = 0; 
				positions[ip + 2] = 0; 
			}
			
		}
	points3DBuffer.geometry.attributes.position.needsUpdate = true;
	points3DBuffer.geometry.attributes.color.needsUpdate = true;
	if (pointSize != null) {
		points3DBuffer.material.uniforms['pointSize'].value = pointSize;
		points3DBuffer.material.needsUpdate = true;
	}
}

this.subscribeTf = function(newTfCallback) {
	this.tfNodes = new Map();
	this.newTfCallback = newTfCallback.bind(this);
	this.tfClient = new ROSLIB.TFClient({
		ros : this.ros,
		fixedFrame : 'base_link',
		angularThres : 0.01,
		transThres : 0.01
	});
	var namespace = this.deviceInfo.get("VIPER_PREFIX");
	this.subscribeTopic('tf', this.onTfUpdate.bind(this));
	this.subscribeTopic('tf_static', this.onTfUpdate.bind(this));
}

this.onTfUpdate = function(msg){
	for(var tf of msg.transforms){
		var parent = tf.header.frame_id;
		if (parent[0] == '/') parent = parent.substring(1);
		if (this.tfNodes.get(parent) == null)
		{
			this.tfNodes.set(parent, new THREE.Group());
			this.onNewTf(parent);
		}
		var child = tf.child_frame_id;
		if (child[0] == '/') child = child.substring(1);
		if (this.tfNodes.get(child) == null)
		{
			this.tfNodes.set(child, new THREE.Group());
			this.onNewTf(child);
		}
		this.tfNodes.get(parent).add(this.tfNodes.get(child));
		
	}
	this.tfClient.processTFArray(msg);
}

this.onNewTf = function(id){
	viper.log('New TF frame, id: ', id);
	var tfUpdateFn = function(tf) {
		this.position.set(tf.translation.x, tf.translation.y, tf.translation.z);
		this.setRotationFromQuaternion(new THREE.Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w));
	}
	this.tfClient.subscribe(id, tfUpdateFn.bind(this.tfNodes.get(id)));
	if (this.newTfCallback != null) this.newTfCallback(id);
}

this.getCvmService = function() {
  return new ROSLIB.Service({
    ros : __instance.ros,
    name : cvmAPIservice
  });
}

this.getConfigNode = function(nodeName) {
	for (var [ttopic, config] of this.dynamicConfig) {
		if (config.node == nodeName)
		{
			return config;
		}
	}
}

this.updateConfgNode = function(cfgNode, callback) {
	var configService = new ROSLIB.Service({
		ros : viper.ros,
		name : cfgNode.node + '/set_parameters',
		serviceType : 'dynamic_reconfigure/Reconfigure'
    });
	var request = new ROSLIB.ServiceRequest({config: cfgNode.cfg});
	
	configService.callService(request, function(result) {
		if (callback != null) callback(result);
    });

}

this.setParameter = function(nodeName, paramName, value, callback){
	var cfgNode = this.getConfigNode(nodeName);
	cfgNode.setParamValue( paramName, value);
	this.updateConfgNode(cfgNode, callback);
}

/** Default VIPER log function. Override to redirect log to different output
 */
this.log = function(m1, m2, m3, m4, m5, m6, m7, m8, m9, m10) {
	console.log('VIPER: '
		,m1 == null? "":m1
		,m2 == null? "":m2
		,m3 == null? "":m3
		,m4 == null? "":m4
		,m5 == null? "":m5
		,m6 == null? "":m6
		,m7 == null? "":m7
		,m8 == null? "":m8
		,m9 == null? "":m9
		,m10 == null? "":m10
		);
}

// Events
this.onConnected = function() {}
this.onDisconnected = function() {}

}// VIPER

// Privates
VIPER.checkStatus = function(viper) {
	var wasConnected = viper.connected;
	var isConnected = true;
	isConnected = isConnected && viper.ros.isConnected;
	isConnected = isConnected && viper.deviceInfo != null;
	isConnected = isConnected && viper.apps != null;
	isConnected = isConnected && viper.topics != null;
	isConnected = isConnected && viper.dynamicConfig != null;
	if (viper.dynamicConfig != null)
		for (var [ttopic, config] of viper.dynamicConfig) {
			isConnected = isConnected && config.desc != null && config.cfg != null;
		}
		
	if (isConnected != wasConnected)
	{
		viper.connected = isConnected;
		if (viper.connected) viper.onConnected();
		else viper.onDisconnected();
	}
}

class ConfigItem
{
	constructor(node)
	{
		this.node = node;
		this.desc = null;
		this.cfg = null;
		
	}
	
	descCallback(desc) {
		this.desc = desc;
		VIPER.checkStatus(__instance);
	}
	
	cfgCallback(cfg) {
		this.cfg = cfg;
		VIPER.checkStatus(__instance);
	}
	
	getAllValues(){
		var ret = [];
		for (var b of this.cfg.bools) ret.push(b);
		for (var b of this.cfg.doubles) ret.push(b);
		for (var b of this.cfg.strs) ret.push(b);
		for (var b of this.cfg.ints) ret.push(b);
		return ret;
	}
	getAllDescriptions(){
		var ret = [];
		for (var b of this.desc.dflt.bools) ret.push({ name: b.name, value: b.value, type: "bool"});
		for (var b of this.desc.dflt.doubles) ret.push({ name: b.name, value: b.value, type: "double"});
		for (var b of this.desc.dflt.strs) ret.push({ name: b.name, value: b.value, type: "string"});
		for (var b of this.desc.dflt.ints) ret.push({ name: b.name, value: b.value, type: "int"});
		return ret;
	}
	
	getParameter(name){
		for (var param of this.getAllValues())
			if (param.name == name) 
				return param;
	}
	
	getParamDesc(name) {
		for (var desc of this.getAllDescriptions())
			if (desc.name == name) 
				return desc;
	}
	
	setParamValue(paramName, value) {
		var param = this.getParameter(paramName);
		var desc = this.getParamDesc(paramName);
		if (desc.type == "double") param.value = parseFloat(value);
		else if (desc.type == "int") param.value = parseInt(value);
		else if (desc.type == "bool") param.value = value == "true";
		else param.value = value;

	}
	
}

VIPER.readDynamicConfig = function(viper) {

	viper.dynamicConfig = new Map();
	
	for (var [ttopic, ttype] of viper.topics) {
		if (ttype == 'dynamic_reconfigure/ConfigDescription') {
			var node = ttopic.replace('/parameter_descriptions', '');
			var topicConfig = viper.dynamicConfig.get(node);
			if (topicConfig == null) { 
				topicConfig = new ConfigItem(node);
				viper.dynamicConfig.set(node, topicConfig);
				//viper.log('Topic ', ttopic, ' is a dynamic configuration description');
			}
			var sub = viper.subscribeTopic(ttopic, topicConfig.descCallback.bind(topicConfig));
		}
		if (ttype == 'dynamic_reconfigure/Config') {
			var node = ttopic.replace('/parameter_updates', '');
			var topicConfig = viper.dynamicConfig.get(node);
			if (topicConfig == null) { 
				topicConfig = new ConfigItem(node);
				viper.dynamicConfig.set(node, topicConfig);
				//viper.log('Topic ', ttopic, ' is a dynamic configuration');
			}
			var sub = viper.subscribeTopic(ttopic, topicConfig.cfgCallback.bind(topicConfig));
		}
	}

}

VIPER.getDeviceInfo = function(viper)
{
	var cvmAPI = viper.getCvmService();

	var request = new ROSLIB.ServiceRequest({
	'auth' : 'labas', 'cmd' : 'info', 'prm1' : '~', 'prm2' : '~', 'prm3' : '~'
	});

	if (viper.isSimulation){
		viper.apps = new Array();
		viper.apps[0] = { name : 'App1', isActive : true };
		viper.version = "Emulator";
		viper.deviceInfo = new Map();
		viper.deviceInfo.set("VIPER_PREFIX", "VIPER");
		VIPER.checkStatus(viper);
	}
	else
	{
		viper.log('Calling CVM API service: info');
		cvmAPI.callService(request, function(result) {
		viper.log('CVM API service responded to request "' + request.cmd + '". RC: '+result.rc);
		var datetime = "" + new Date().toLocaleString();
		viper.version = result.version;
		viper.apps = result.apps;
		viper.activeApps = result.activeApps;

		viper.deviceInfo = new Map();
		for(var i = 0; i < result.info.length; i++) {
			var key = result.info[i].substring(0, result.info[i].indexOf("="));
			var value = result.info[i].substring(key.length + 1);
			viper.deviceInfo.set(key, value);
		}
		VIPER.checkStatus(__instance);
		});
	}
}

VIPER.getTopics = function(viper) {
	var topicsClient = new ROSLIB.Service({
		ros : viper.ros,
		name : '/rosapi/topics',
		serviceType : 'rosapi/Topics'
    });

    var request = new ROSLIB.ServiceRequest();
	viper.log("Getting topics...");

    topicsClient.callService(request, function(result) {
		viper.topics = new Map();
		viper.log('Topics received (', result.topics.length + ')');
		for (var i = 0; i < result.topics.length; i++)
			viper.topics.set(result.topics[i], result.types[i]);
		viper.topics = new Map([...viper.topics].sort((a, b) => a[0] < b[0]? -1 : 1));
		VIPER.readDynamicConfig(viper);
		VIPER.checkStatus(viper);
    });
}

VIPER.createViper3DModel = function() {
	var root = new THREE.Group();
	var material = new THREE.MeshStandardMaterial( {color: 0xaa0000, metalness: 0.7} );
	var geometry = new THREE.BoxGeometry( 0.068, 0.246, 0.035 );
	var box = new THREE.Mesh( geometry, material );
	box.position = new THREE.Vector3( 0, 0, 0 );
	root.add( box );

	geometry = new THREE.BoxGeometry( 0.032,0.15, 0.025 );
	box = new THREE.Mesh( geometry, material );
	box.position.set( -0.09/2, 0, 0 );
	root.add( box );

	geometry = new THREE.BoxGeometry( 0.05,0.14, 0.01 );
	box = new THREE.Mesh( geometry, material );
	box.position.set( -0.02, 0, -0.035/2 );
	root.add( box );
	
	var box = new THREE.Box3();
	box.setFromCenterAndSize( new THREE.Vector3( 0, 0, 0 ), new THREE.Vector3( 0.068, 0.246, 0.035 ) );
	var helper = new THREE.Box3Helper( box, 0xaaaaaa );
	root.add( helper );
	
	box = new THREE.Box3();
	box.setFromCenterAndSize( new THREE.Vector3( -0.09/2, 0, 0 ), new THREE.Vector3( 0.032,0.15, 0.025 ) );
	helper = new THREE.Box3Helper( box, 0xaaaaaa );
	root.add( helper );

	box = new THREE.Box3();
	box.setFromCenterAndSize( new THREE.Vector3( -0.02, 0, -0.035/2), new THREE.Vector3( 0.05,0.14, 0.01 ) );
	helper = new THREE.Box3Helper( box, 0xaaaaaa );
	root.add( helper );
	return root;
}

VIPER.createDefault3DViewer = function(options) {
	options = options || {};
	var divElement = options.divElement;
	var width = options.width;
	var height = options.height;
	var addAxis = (options.addAxis == null) || options.addAxis && true;
	var showOverlay = (options.showOverlay == null) || options.showOverlay && true;
	
	var viewer = new ViperViewer({
	//new ROS3D.Viewer({
		divID : divElement,
		width : width,
		height : height,
		antialias : true,
		background : "#888888",
		displayPanAndZoomFrame: true,
		lineTypePanAndZoomFrame: 'full'
	});
	
	// Camera
	viewer.camera.position.set(4, 3, -5);
	viewer.camera.up = new THREE.Vector3(0, 1, 0);
	viewer.camera.lookAt(new THREE.Vector3(0, 0, 0));
	
	viewer.cameraControls.screenSpacePanning = true;
	viewer.cameraControls.enableKeys = true;

	var minorGrid = new THREE.GridHelper(10, 50, 0xDDDDDD, 0x999999);
	viewer._sceneRoot.add( minorGrid);
	minorGrid.position.y = -0.011;
	var majorGrid = new THREE.GridHelper(20, 20, 0xDDDDDD, 0x777777);
	viewer._sceneRoot.add( majorGrid);
	majorGrid.position.y = -0.01;
	
	if (addAxis == true) {
		var axesHelper = new THREE.AxesHelper( 1 );
		viewer.scene.add( axesHelper );
	}
	if (showOverlay) {
		var overlayAxes = new THREE.AxesHelper( 1 );
		overlayAxes.rotation.set(-Math.PI/2, 0, -Math.PI/2);	// This transforms from WebGL to ROS coordinate systems.
		viewer.overlayScene.add( overlayAxes );
		viewer.cameraControls.addEventListener( 'change', function(){
			viewer.overlayCamera.position.set(0, 0, viewer.camera.position.z);
			var q = viewer.camera.quaternion.clone();
			viewer.overlayCamera.rotation.setFromQuaternion(viewer.camera.quaternion);
			var pos = new THREE.Vector3(0, 0, 3);			
			pos.applyQuaternion(q);
			viewer.overlayCamera.position.set(pos.x, pos.y, pos.z);
			viewer.overlayCamera.updateProjectionMatrix();

			}
		);
	}

	var material = new THREE.LineBasicMaterial({ color: 0x0000ff });

	var geometry = new THREE.Geometry();
	geometry.vertices.push(new THREE.Vector3(5, 5, 10));
	geometry.vertices.push(new THREE.Vector3(0, 0, 0));
	geometry.vertices.push(new THREE.Vector3(-5, 5, 10));
	geometry.vertices.push(new THREE.Vector3(0, 0, 0));
	geometry.vertices.push(new THREE.Vector3(5, -5, 10));
	geometry.vertices.push(new THREE.Vector3(0, 0, 0));
	geometry.vertices.push(new THREE.Vector3(-5, -5, 10));

	var line = new THREE.Line(geometry, material);

	//viewer.scene.add(line);
	//var directionalLight = new THREE.DirectionalLight( 0xffffff, 0.5 );
	//viewer.scene.add( directionalLight );
	
	line.rotation.y = 3.14/2;
	line.position.z = 5;
	return viewer;
}

VIPER.pointCloudVertexShader = function() {
	return `
	varying vec3 vColor;
	uniform float pointSize;
	
	void main()	{
		vColor = color;
		vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );
		gl_PointSize = pointSize;
		gl_Position = projectionMatrix * mvPosition;
	}
	`
}

VIPER.pointCloudFragmentShader = function() {
	return `
	varying vec3 vColor;
	void main() {
		if ( length( gl_PointCoord - vec2( 0.5, 0.5 ) ) > 0.475 ) discard;
		gl_FragColor = vec4( vColor.x, vColor.y, vColor.z, 1.0 );
		}
	`
}


/* Function converts vector from ROS to GL convention
 */
VIPER.vec3Ros2Gl = function(ros) {
	var rotMat = new THREE.Matrix4();
	rotMat.makeRotationFromEuler(new THREE.Euler(-Math.PI/2, 0, -Math.PI/2));
	var ret = ros.clone();
	ret.applyMatrix4(rotMat);
	return ret;
}

/* Function converts vector from GL to ROS convention
 */
VIPER.vec3Gl2Ros = function(webGl) {
	var rotMat = new THREE.Matrix4();
	rotMat.makeRotationFromEuler(new THREE.Euler(-Math.PI/2, 0, -Math.PI/2));
	rotMat.getInverse(rotMat);
	var ret = webGl.clone();
	ret.applyMatrix4(rotMat);
	return ret;
}

VIPER.mat4Ros2Gl = function(ros) {
	var rotMat = new THREE.Matrix4();
	rotMat.makeRotationFromEuler(new THREE.Euler(-Math.PI/2, 0, -Math.PI/2));
	return rotMat.multiply(ros.clone());
}

VIPER.mat4Gl2Ros = function(webGl) {
	var rotMat = new THREE.Matrix4();
	rotMat.makeRotationFromEuler(new THREE.Euler(-Math.PI/2, 0, -Math.PI/2));
	rotMat.getInverse(rotMat);
	return rotMat.multiply(webGl.clone());
}

/** Class which creates 3D view from div element and customizes basic scene elements light lighting, grid and view controls
 */
class ViperViewer
{
	
	constructor(options) {
		options = options || {};
		var divID = options.divID;
		var width = options.width;
		var height = options.height;
		var background = options.background || '#111111';
		var antialias = options.antialias;
		var intensity = options.intensity || 0.66;
		var near = options.near || 0.01;
		var far = options.far || 1000;
		var alpha = options.alpha || 1.0;
		var cameraPosition = options.cameraPose || {
		  x : 3,
		  y : 3,
		  z : 3
		};
		var cameraZoomSpeed = options.cameraZoomSpeed || 0.5;
		var displayPanAndZoomFrame = (options.displayPanAndZoomFrame === undefined) ? true : !!options.displayPanAndZoomFrame;
		var lineTypePanAndZoomFrame = options.lineTypePanAndZoomFrame || 'full';

		if (this.renderer == null) {
			// create the canvas to render to
			this.renderer = new THREE.WebGLRenderer({
			  antialias : antialias,
			  alpha: true
			});
		}
		this.renderer.setClearColor(parseInt(background.replace('#', '0x'), 16), alpha);
		this.renderer.sortObjects = false;
		this.renderer.setSize(width, height);
		this.renderer.shadowMap.enabled = false;
		this.renderer.autoClear = false;
		
		// Overlay part for axes helper
		var oWidth = width/8;
		var oHeight = height/8;
		this.overlayRenderer = new THREE.WebGLRenderer({
			  antialias : antialias,
			  alpha: true
			});
		this.overlayRenderer.setClearColor(parseInt(background.replace('#', '0x'), 16), alpha);
		this.overlayRenderer.sortObjects = false;
		this.overlayRenderer.setSize(oWidth, oHeight);
		this.overlayRenderer.shadowMap.enabled = false;
		this.overlayRenderer.autoClear = false;

		// create the global scene
		this._sceneRoot = new THREE.Scene();
		this.scene = new THREE.Group();
		this._sceneRoot.add(this.scene);
		this.scene.rotation.set(-Math.PI/2, 0, -Math.PI/2);	// This is required to transform from WebGL to ROS coordinate systems


		// create the global camera
		this.camera = new THREE.PerspectiveCamera(40, width / height, near, far);
		this.camera.position.x = cameraPosition.x;
		this.camera.position.y = cameraPosition.y;
		this.camera.position.z = cameraPosition.z;
		// add controls to the camera
		this.cameraControls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
		this.cameraControls.userZoomSpeed = cameraZoomSpeed;
		
		this.overlayCamera = new THREE.PerspectiveCamera(40, width / height, near, far);
		this.overlayCamera.position.set(0, 0, 10);
		this.overlayScene = new THREE.Scene();

		// lights
		this.scene.add(new THREE.AmbientLight(0x555555));
		this.directionalLight = new THREE.DirectionalLight(0xffffff, intensity);
		this.scene.add(this.directionalLight);
		this.directionalLight.position.x = -0.3;
		this.directionalLight.position.y = -0.2;
		this.directionalLight.position.z = 1.0;
		this.directionalLight.lookAt(0, 0, 0);

		// propagates mouse events to three.js objects
		this.selectableObjects = new THREE.Object3D();
		this._sceneRoot.add(this.selectableObjects);

		this.stopped = true;
		this.animationRequestId = undefined;

		// add the renderer to the page
		var mainRenderer = document.createElement("div");
		var overlayRenderer = document.createElement("div");
		document.getElementById(divID).setAttribute("style", "position: relative");
		overlayRenderer.setAttribute("style", "position:absolute;top:0;left:0;z-index: 10;width:" + oWidth + "px;height:"+ oHeight +"px");
		document.getElementById(divID).appendChild(mainRenderer);
		document.getElementById(divID).appendChild(overlayRenderer);
		mainRenderer.appendChild(this.renderer.domElement);
		overlayRenderer.appendChild(this.overlayRenderer.domElement);
		
		// begin the render loop
		this.start();
	}
	
	  /**
   *  Start the render loop
   */
	start(){
		this.stopped = false;
		this.draw();
	};

	/**
	* Renders the associated scene to the viewer.
	*/
	draw(){
		if(this.stopped){
		  // Do nothing if stopped
		  return;
		}

		// update the controls
		this.cameraControls.update();

		// put light to the top-left of the camera
		// BUG: position is a read-only property of DirectionalLight,
		// attempting to assign to it either does nothing or throws an error.
		//this.directionalLight.position = this.camera.localToWorld(new THREE.Vector3(-1, 1, 0));
		this.directionalLight.position.normalize();

		// set the scene
		this.renderer.clear(true, true, true);
		this.renderer.render(this._sceneRoot, this.camera);
		//this.renderer.clearDepth();
		//this.overlayRenderer.clear(true, true, true);
		this.overlayRenderer.render(this.overlayScene, this.overlayCamera);
		//this.renderer.setSize(target.x, target.y);
		//this.highlighter.renderHighlights(this.scene, this.renderer, this.camera);

		// draw the frame
		this.animationRequestId = requestAnimationFrame(this.draw.bind(this));
	};

	/**
	*  Stop the render loop
	*/
	stop(){
		if(!this.stopped){
		  // Stop animation render loop
		  cancelAnimationFrame(this.animationRequestId);
		}
		this.stopped = true;
	};

}