// ROS Connection
const ros = new ROSLIB.Ros({
    url: 'http://localhost:9090'  // Change this to your ROSBridge WebSocket URL
});

ros.on('connection', function () {
    console.log('Connected to ROS');
});

ros.on('error', function (error) {
    console.log('Error connecting to ROS:', error);
});

ros.on('close', function () {
    console.log('Connection to ROS closed');
});

// Define the topic to publish joystick data
const joystickTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/joystick',
    messageType: 'control/Joystick'
});

// Function to detect gamepads
function scanGamepads() {
    const gamepads = navigator.getGamepads();
    console.log(gamepads)
    return gamepads[0]; // Use the first connected gamepad
}

// Send joystick data to ROS
function sendJoystickData() {
    const gamepad = scanGamepads();
    if (!gamepad) {
        return;
    }

    // Extract joystick axes
    const axes = gamepad.axes.map(value => parseFloat(value.toFixed(2)));
    const buttons = gamepad.buttons.map(btn => btn.pressed ? 1 : 0);
    const button_start = !!buttons[9]
    const button_share = !!buttons[8]
    const button_touch = !!buttons[16]

    // Extract button presses
    console.log(axes)
    console.log(buttons)
    // Create and send ROS message
    const joystickMessage = new ROSLIB.Message({
        left_x_axis: axes[0],
        left_y_axis: axes[1],
        right_x_axis: axes[2],
        right_y_axis: axes[3],
        button_x: !!buttons[0],
        button_o: !!buttons[1],
        button_rect: !!buttons[2],
        button_tri: !!buttons[3],
        button_l1: !!buttons[4],
        button_r1: !!buttons[5],
        button_l2: !!buttons[6],
        button_r2: !!buttons[7],
        button_l3: !!buttons[10],
        button_r3: !!buttons[11],
        button_top: !!buttons[12],
        button_bot: !!buttons[13],
        button_left: !!buttons[14],
        button_right: !!buttons[15],
        button_options: !!buttons[9],
        button_share: !!buttons[8]
    });

    joystickTopic.publish(joystickMessage);
}

// Poll joystick data every 100ms
setInterval(sendJoystickData, 100);

var pidTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/constants',
    messageType: 'std_msgs/Float32MultiArray'
});

// Function to send PID constants via ROS topic
function sendPIDConstants() {
    var kp1 = parseFloat(document.getElementById("kp1").value) || 0;
    var ki1 = parseFloat(document.getElementById("ki1").value) || 0;
    var kd1 = parseFloat(document.getElementById("kd1").value) || 0;
    var kp2 = parseFloat(document.getElementById("kp2").value) || 0;
    var ki2 = parseFloat(document.getElementById("ki2").value) || 0;
    var kd2 = parseFloat(document.getElementById("kd2").value) || 0;
    var kp3 = parseFloat(document.getElementById("kp3").value) || 0;
    var ki3 = parseFloat(document.getElementById("ki3").value) || 0;
    var kd3 = parseFloat(document.getElementById("kd3").value) || 0;

    var pidMessage = new ROSLIB.Message({
        data: [kp1, ki1, kd1, kp2, ki2, kd2, kp3, ki3, kd3]
    });

    pidTopic.publish(pidMessage);
    console.log("Sent PID constants:", pidMessage.data);
}