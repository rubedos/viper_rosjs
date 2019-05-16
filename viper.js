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
	  //this.ros.socket.close();
      this.ros.close();
      this.ros = null;
    }
	this.log('Closing connection to ' + this.ros_ip);
}

this.getTopicType = function(topic) {
	if (this.topics == null) throw "Topic list is empty";
	var type = null;
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
		callback({rgb:rgbBuffer, disparity: disparityBuffer, width: w, height: h, focal: focal, baseline: baseline, 
			dispMin: message.disparityImage.min_disparity, dispMax: message.disparityImage.max_disparity});
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
			color: { value: new THREE.Color( 0xffffff ) },
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

/** Renders received disparity buffer onto canvas element
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

/** This function uses ros PointCloud2 buffer data to update 3d model of the pointcloud
 */
this.updatePointsBuffer = function(rosBuffer, points3DBuffer, w, h) {
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
				if (x > 5) x = 5;
				positions[ip] = y; 
				positions[ip + 1] = z; 
				positions[ip + 2] = x; 
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
		VIPER.checkStatus(viper);
    });
}


VIPER.createDefault3DViewer = function(divElement, width, height) {
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
	viewer.scene.add( minorGrid);
	minorGrid.position.y = -0.01;
	var majorGrid = new THREE.GridHelper(20, 20, 0xDDDDDD, 0x777777);
	viewer.scene.add( majorGrid);
	
	var axesHelper = new THREE.AxesHelper( 1 );
	viewer.scene.add( axesHelper );

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
	void main() {
		vColor = color;
		vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );
		gl_PointSize = 1.0;//scale * ( 300.0 / - mvPosition.z );
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

		// create the global scene
		this.scene = new THREE.Scene();

		// create the global camera
		this.camera = new THREE.PerspectiveCamera(40, width / height, near, far);
		this.camera.position.x = cameraPosition.x;
		this.camera.position.y = cameraPosition.y;
		this.camera.position.z = cameraPosition.z;
		// add controls to the camera
		this.cameraControls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
		this.cameraControls.userZoomSpeed = cameraZoomSpeed;

		// lights
		this.scene.add(new THREE.AmbientLight(0x555555));
		this.directionalLight = new THREE.DirectionalLight(0xffffff, intensity);
		this.scene.add(this.directionalLight);
		this.directionalLight.position.x = -0.3;
		this.directionalLight.position.z = -0.2;
		this.ambientLight = new THREE.AmbientLight( 0x404040 ); // soft white light
		this.scene.add( this.ambientLight );

		// propagates mouse events to three.js objects
		this.selectableObjects = new THREE.Object3D();
		this.scene.add(this.selectableObjects);
		/*var mouseHandler = new MouseHandler({
		  renderer : this.renderer,
		  camera : this.camera,
		  rootObject : this.selectableObjects,
		  fallbackTarget : this.cameraControls
		});

		// highlights the receiver of mouse events
		this.highlighter = new Highlighter({
		  mouseHandler : mouseHandler
		});*/

		this.stopped = true;
		this.animationRequestId = undefined;

		// add the renderer to the page
		document.getElementById(divID).appendChild(this.renderer.domElement);
		
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
		this.renderer.render(this.scene, this.camera);
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