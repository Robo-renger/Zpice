// ROS Connection
const ros = new ROSLIB.Ros({
    url: '192.168.1.233:9090'  // Change this to your ROSBridge WebSocket URL
});

ros.on('connection', function() {
    console.log('Connected to ROS');
});

ros.on('error', function(error) {
    console.log('Error connecting to ROS:', error);
});

ros.on('close', function() {
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
    return gamepads[0]; // Use the first connected gamepad
}

// Send joystick data to ROS
function sendJoystickData() {
    const gamepad = scanGamepads();
    if (!gamepad) {
        return;
    }

    // Extract joystick axes
    const axes = gamepad.axes.map(value => parseFloat(value.toFixed(2))); // Normalize to 2 decimal places

    // Extract button presses
    const buttons = gamepad.buttons.map(btn => btn.pressed ? 1 : 0);

    // Create and send ROS message
    const joystickMessage = new ROSLIB.Message({
        axes: axes,
        buttons: buttons
    });

    joystickTopic.publish(joystickMessage);
}

// Poll joystick data every 100ms
setInterval(sendJoystickData, 100);
