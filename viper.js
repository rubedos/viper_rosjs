/**
 * @author Henrikas Simonavičius / henrikas.simonavicius@rubedos.com
 * @author Rubedos / http://www.rubedos.com
 */


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
      //this.connected = false;
      VIPER.checkStatus(__instance);
    });

    this.ros.on('close', function() {
      __instance.log('Connection to websocket server closed.');
      //this.connected = false;
      VIPER.checkStatus(__instance);
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
      this.ros.close();
      this.ros = null;
    }
	this.log('Closed connection to ' + this.ros_ip);
}

/** Subscribe to image topic and sends updated image data to given callback.
 Function returns listener which is a handle to call unsubscribe when finished streaming.
 @param topic - the name of image topic
 @param callback(rgbByteBuffer, width, height) - a function to be called with updated image data (RGB byte buffer, image width, image height).
 @return subscription listener
 */
this.subscribeImage = function(topic, callback){
	__instance.log('Subscribing to topic ', topic);
	var listener = new ROSLIB.Topic({
		ros : __instance.ros,
		name : topic,
		messageType : 'sensor_msgs/Image',
		queue_size : 1
	});
	listeners.set(listener.name, callback);
	listener.subscribe(function(message) {
	var w = message.width;
	var h = message.height;
	var buffer = new Uint8Array(Base64Binary.decodeArrayBuffer(message.data));
	callback(buffer, w, h);
  });

  return listener;
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

this.getCvmService = function() {
  return new ROSLIB.Service({
    ros : __instance.ros,
    name : cvmAPIservice
  });
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
this.onDisonnected = function() {}



}// VIPER

// Privates
VIPER.checkStatus = function(viper) {
	var wasConnected = viper.connected;
	var isConnected = true;
	isConnected = isConnected && viper.ros.isConnected;
	isConnected = isConnected && viper.deviceInfo != null;
	isConnected = isConnected && viper.apps != null;
	isConnected = isConnected && viper.topics != null;
	
	if (isConnected != wasConnected)
	{
		viper.connected = isConnected;
		if (viper.connected) viper.onConnected();
		else viper.onDisconnected();
	}
}

VIPER.getDeviceInfo = function(viper)
{
  var cvmAPI = viper.getCvmService();

  var request = new ROSLIB.ServiceRequest({
    'auth' : 'labas', 'cmd' : 'info', 'prm1' : '~', 'prm2' : '~', 'prm3' : '~'
  });

  viper.log('Calling CVM API service: info');
  cvmAPI.callService(request, function(result) {
    viper.log('CVM API service responded. RC: '+result.rc);
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

VIPER.getTopics = function(viper) {
	var topicsClient = new ROSLIB.Service({
		ros : viper.ros,
		name : '/rosapi/topics',
		serviceType : 'rosapi/Topics'
    });

    var request = new ROSLIB.ServiceRequest();
	viper.log("Getting topics...");

    topicsClient.callService(request, function(result) {
		viper.log('Topics received (', result.topics.length + ')');
		__instance.topics = result.topics;
		VIPER.checkStatus(__instance);
    });
}

