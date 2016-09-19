/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * Manages connection to the server and all interactions with ROS.
 *
 * Emits the following events:
 *   * 'change' - emitted with a change in speed occurs
 *
 * @constructor
 * @param options - possible keys include:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the Twist topic to publish to, like '/cmd_vel'
 *   * throttle (optional) - a constant throttle for the speed
 */

var KEYBOARDTELEOP = KEYBOARDTELEOP || { REVISION:"0.3.0" };

KEYBOARDTELEOP.Teleop = function(options) {
    var that = this;
    options = options || {};
    var ros = options.ros;
    var topic = options.topic || '/cmd_vel';
    // permanent throttle
    var throttle = options.throttle || 1.0;

    // used to externally throttle the speed (e.g., from a slider)
    this.scale = 1.0;

    // linear x and y movement and angular z movement
    var x = 0;
    var y = 0;
    var z = 0;

    var cmdVel = new ROSLIB.Topic({
	ros : ros,
	name : topic,
	messageType : 'geometry_msgs/Twist'
    });

    // sets up a key listener on the page used for keyboard teleoperation
    var handleKey = function(keyCode, keyDown) {
	// used to check for changes in speed
	var oldX = x;
	var oldY = y;
	var oldZ = z;

	var pub = true;

	var speed = 0;
	// throttle the speed by the slider and throttle constant
	speed = throttle * that.scale;
	
	// check which key was pressed
	switch (keyCode) {
	case 74:
	    // turn left
	    z = 1 * speed;
	    break;
	case 73:
	    // up
	    x = 1.0 * speed;
	    break;
	case 76:
	    // turn right
	    z = -1 * speed;
	    break;
	case 188:
	    // down
	    x = -1.0 * speed;
	    break;
	case 85:
	    // forward + left
	    x = 1.0 * speed;
	    z = 1.0 * speed;
	    break;
	case 79:
	    // forward + right
	    x = 1.0 * speed;
	    z = -1.0 * speed;
	    break;
	default:
	    // pub = false;
	    x = y = z = 0;
	}

	// publish the command
	if (pub === true) {
	    var twist = new ROSLIB.Message({
		angular : {
		    x : 0,
		    y : 0,
		    z : z
		},
		linear : {
		    x : x,
		    y : 0,
		    z : 0
		}
	    });
	    cmdVel.publish(twist);

	    // check for changes
	    if (oldX !== x || oldY !== y || oldZ !== z) {
		that.emit('change', twist);
	    }
	}
    };

    // handle the key
    var body = document.getElementsByTagName('body')[0];
    body.addEventListener('keydown', function(e) {
	handleKey(e.keyCode, true);
    }, false);
    // body.addEventListener('keyup', function(e) {
    // 	handleKey(e.keyCode, false);
    // }, false);
}, KEYBOARDTELEOP.Teleop.prototype.__proto__ = EventEmitter2.prototype;
