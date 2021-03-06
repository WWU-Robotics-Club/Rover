// from http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality

// Connecting to ROS
// -----------------
var rosWsUrl = 'wss://' + window.location.hostname + '/ws'

var ros = new ROSLIB.Ros({
    url: rosWsUrl
});

ros.on('connection', function () {
    console.log('Connected to websocket server.');
    document.getElementById("websocketStatus").textContent = "Connected";
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
    document.getElementById("websocketStatus").textContent = "Error";
});

ros.on('close', function () {
    console.log('Connection to websocket server closed.');
    document.getElementById("websocketStatus").textContent = "Closed";
});

function rosReconnect() {
    ros.connect(rosWsUrl);
}

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

function rosRefreshStatus() {
    // Get topics list
    ros.getTopics(function(data) {
        el = document.getElementById("topicsList");
        el.innerHTML = "";
        for (var i=0; i < data.topics.length; i++) {
            var item = document.createElement("li");
            item.textContent = data.topics[i];
            el.appendChild(item);
        } 
    });

    // Get nodes list
    ros.getNodes(function(data) {
        el = document.getElementById("nodeList");
        el.innerHTML = "";
        for (var i=0; i < data.length; i++) {
            var item = document.createElement("li");
            item.textContent = data[i];
            el.appendChild(item);
        } 
    });

    // List params
    ros.getParams(function (params) {
        console.log(params);
    });
}

var maxVelX = new ROSLIB.Param({
    ros: ros,
    name: 'max_vel_y'
});

var joystickMode = "rotate"; // "translate";

function changeJoystickMode(target) {
    joystickMode = target.value;
    console.log(joystickMode);
}

createJoystick = function () {
    var options = {
        zone: document.getElementById('joystick1'),
        threshold: 0.1,
        position: { left: 50 + '%', top: 50 + '%' },
        mode: 'static',
        size: 150,
        color: '#000000',
    };
    var manager = nipplejs.create(options);

    var forward_speed = 0;
    var left_speed = 0;
    var angular_speed = 0;
    var timer;

    manager.on('start', function (event, nipple) {
        console.log("Movement start");
        // start sending updates at 10hz
        timer = setInterval(function () {
            move(forward_speed, left_speed, angular_speed);
        }, 200);
    });

    // update inear_speed and angular_speed whenever the joystick moves
    manager.on('move', function (event, nipple) {
        max_linear = 0.7; // m/s
        max_angular = 1.5; // rad/s
        max_distance = 75.0; // pixels;
        forward_speed = Math.sin(nipple.angle.radian) * max_linear * nipple.distance/max_distance;
        left_speed = -Math.cos(nipple.angle.radian) * max_linear * nipple.distance/max_distance;
        angular_speed = -Math.cos(nipple.angle.radian) * max_angular * nipple.distance/max_distance;

        if (joystickMode === "rotate") {
            left_speed = 0;
        } else if (joystickMode === "translate") {
            angular_speed = 0;
        }
    });

    manager.on('end', function () {
        console.log("Movement end");
        if (timer) {
            clearInterval(timer);
        }
        move(0, 0, 0);
    });
}

window.onload = function () {
    createJoystick();
}

// Publishing a Topic
// ------------------

var webVelTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/wheels/cmd_velocity',
    messageType: 'geometry_msgs/Twist'
});


move = function (forward, left, angular) {
    var twist = new ROSLIB.Message({
        linear: {
            x: forward,
            y: left,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: angular
        }
    });
    webVelTopic.publish(twist);
    
    document.getElementById("joystick_forward").textContent = forward.toFixed(2);
    document.getElementById("joystick_left").textContent = left.toFixed(2);
    document.getElementById("joystick_angular").textContent = angular.toFixed(2);
}


// Getting and setting a param value
// ---------------------------------

ros.getParams(function (params) {
    console.log(params);
});

maxVelX.set(0.8);
maxVelX.get(function (value) {
    console.log('MAX VAL: ' + value);
});