// from http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality

// Connecting to ROS
// -----------------

var ros = new ROSLIB.Ros({
    url: 'wss://' + window.location.hostname + '/ws'
});

ros.on('connection', function () {
    console.log('Connected to websocket server.');
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function () {
    console.log('Connection to websocket server closed.');
});

// Subscribing to a Topic
// ----------------------

var listenerTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/listener',
    messageType: 'std_msgs/String'
});

listenerTopic.subscribe(function (message) {
    console.log('Received message on ' + listenerTopic.name + ': ' + message.data);
    listenerTopic.unsubscribe();
});

// Getting and setting a param value
// ---------------------------------

ros.getParams(function (params) {
    console.log(params);
});

var maxVelX = new ROSLIB.Param({
    ros: ros,
    name: 'max_vel_y'
});

maxVelX.set(0.8);
maxVelX.get(function (value) {
    console.log('MAX VAL: ' + value);
});


createJoystick = function () {
    var options = {
        zone: document.getElementById('zone_joystick'),
        threshold: 0.1,
        position: { left: 50 + '%' },
        mode: 'static',
        size: 150,
        color: '#000000',
    };
    var manager = nipplejs.create(options);

    var linear_speed = 0;
    var angular_speed = 0;
    var timer;


    manager.on('start', function (event, nipple) {
        console.log("Movement start");
        // start sending updates at 10hz
        timer = setInterval(function () {
            move(linear_speed, angular_speed);
        }, 200);
    });

    // update inear_speed and angular_speed whenever the joystick moves
    manager.on('move', function (event, nipple) {
        max_linear = 5.0; // m/s
        max_angular = 2.0; // rad/s
        max_distance = 75.0; // pixels;
        linear_speed = Math.sin(nipple.angle.radian) * max_linear * nipple.distance/max_distance;
        angular_speed = -Math.cos(nipple.angle.radian) * max_angular * nipple.distance/max_distance;
    });

    manager.on('end', function () {
        console.log("Movement end");
        if (timer) {
            clearInterval(timer);
        }
        self.move(0, 0);
    });
}

window.onload = function () {
    createJoystick();
}

// Publishing a Topic
// ------------------

var webVelTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/web_vel',
    messageType: 'geometry_msgs/Twist'
});


move = function (linear, angular) {
    var twist = new ROSLIB.Message({
        linear: {
            x: linear,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: angular
        }
    });
    webVelTopic.publish(twist);
}