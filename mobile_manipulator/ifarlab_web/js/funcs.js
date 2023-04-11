function slctPose() {
  var poselist = document.getElementById("mrposition");
  var index = poselist.options[poselist.selectedIndex].value;
  if (index == "Task") {
    document.getElementById("mrposition_taskid").style.display = "block";
  }
  else {
    document.getElementById("mrposition_taskid").style.display = "none";
  }
}

document.getElementById("mr_ui_sensor").value = 2.457

// Get the modal
var emgmodal = document.getElementById("emgModal");
emgmodal.style.display = "none";

// Get the modal
var commodal = document.getElementById("comModal");

// Get the button that opens the modal
var btn = document.getElementById("emergency_button");

// Get the <span> element that closes the modal
var span = document.getElementsByClassName("close")[0];

// When the user clicks the button, open the modal 
btn.onclick = function () {
  emgmodal.style.display = "block";
  emg_stop();
}

// When the user clicks on <span> (x), close the modal
span.onclick = function () {
  emgmodal.style.display = "none";
}

// When the user clicks anywhere outside of the modal, close it
window.onclick = function (event) {
  if (event.target == emgmodal) {
    emgmodal.style.display = "none";
  }
}

// Connecting to ROS
// -----------------

  // Connect to ROS.
ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});



let testBool = 0;

ros.on('connection', function () {
  console.log("Connected to websocket server")
  isConnected=true;
}) ;

ros.on('close', function (error) {
console.log("Close connecting to websocket server:", error);  
isConnected=false
commodal.style.display = "block";
});

ros.on('error', function (error) {
  console.log("Error connecting to websocket server:", error);  
  isConnected=false
  commodal.style.display = "block";

});



var ui_start_button = 0;

// Publishing a Topic
// ------------------

var ui_start = new ROSLIB.Topic({
  ros: ros,
  name: '/ui_start',
  messageType: 'std_msgs/Int8'
});

var ui_emg_stop = new ROSLIB.Topic({
  ros: ros,
  name: '/ui_emg',
  messageType: 'std_msgs/Int8'
});

var emg_sub = new ROSLIB.Topic({
  ros: ros,
  name: '/emg',
  messageType: 'std_msgs/Int8'
});

var ui_start = new ROSLIB.Topic({
  ros: ros,
  name: '/ui_start',
  messageType: 'std_msgs/Int8'
});

var sensor = new ROSLIB.Topic({
  ros: ros,
  name: '/low_laser',
  messageType: 'std_msgs/String'
});

var ui_status = new ROSLIB.Topic({
  ros: ros,
  name: '/ui_task_status',
  messageType: 'std_msgs/String'
});

var mobile_status_topic = new ROSLIB.Topic({
  ros: ros,
  name: '/mobile_status',
  messageType: 'std_msgs/String'
});

function start() {
  ui_start_button = 1;
  var ui_start_data = new ROSLIB.Message({
    data: ui_start_button
  });

  console.log(ui_start_button)
  ui_start.publish(ui_start_data);
  //<!-- ui_start_button = 0; -->
}

function emg_stop() {
  var emg_stop_data = new ROSLIB.Message({
    data: 1
  });

  ui_emg_stop.publish(emg_stop_data);
}

document.getElementById("mr_motion").style.pointerEvents = "none";
document.getElementById("mr_motion").style.opacity = 0.5;

ui_status_flag = false
ui_status.subscribe(function (message) {
  let text = (message.data).replace(/['"]+/g, '"');
  json_data = JSON.parse(text);
  task_status_data = json_data.task_status
  active_id_data = json_data.active_id
  // console.log(json_data)
  document.getElementById("task_status").value = task_status_data;
  document.getElementById("task_id").value = active_id_data;
  if (task_status_data.includes("Manuel")){
    if (!ui_status_flag){
      testBool = 1
      document.getElementById("mr_motion").style.pointerEvents = "auto";
      document.getElementById("mr_motion").style.opacity = 1.0;
      document.getElementById("tasktracking").style.pointerEvents = "none";
      document.getElementById("tasktracking").style.opacity = 0.5;
      document.getElementById("toggle-button").checked = true;
      ui_status_flag = true
    }
  }
  else{
    if (ui_status_flag){
    testBool = 0
    document.getElementById("mr_motion").style.pointerEvents = "none";
    document.getElementById("mr_motion").style.opacity = 0.5;
    document.getElementById("toggle-button").checked = false;
    document.getElementById("tasktracking").style.pointerEvents = "auto";
    document.getElementById("tasktracking").style.opacity = 1.0;
    ui_status_flag = false
    }
  }
});

sensor.subscribe(function (message) {
  let text = (message.data).replace(/['"]+/g, '"');
  json_data = JSON.parse(text);
  sensor_data = json_data.Distance_MM / 1000;
  document.getElementById("mr_sensor").value = sensor_data;
});

ui_emg_stop.subscribe(function (message) {
  console.log(message.data)
  if (message.data == 1) {
    emgmodal.style.display = "block";
  }
  else {
    emgmodal.style.display = "none";
  }
});

emg_sub.subscribe(function (message) {
  if (message.data == 1) {
    if (emgmodal.style.display == "none"){
      emgmodal.style.display = "block";
    }
  }
});

mobile_status_topic.subscribe(function (message) {
  document.getElementById("text_mr_status").value = message.data;
});
// Calling a service
// -----------------

var addTwoIntsClient = new ROSLIB.Service({
  ros: ros,
  name: '/right_rokos_task_service',
  serviceType: 'srvt_moveit/TaskService'
});

addTwoIntsClient.callService(" ", function (result) {
  var task_list = []
  let list = document.getElementById("tasklist");
  console.log('Result for service call on '
    + addTwoIntsClient.name
    + ': '
    + result.response);
  var myJSON = eval(eval(JSON.stringify(result.response)));
  for (let x of myJSON) {
    task_list.push(x[x.length - 1][0])
    let li = document.createElement("li");
    li.innerText = x[x.length - 1][0];
    list.appendChild(li);
  }
  console.log(task_list)

});

var odom_sub = new ROSLIB.Topic({
  ros: ros,
  name: '/odom',
  messageType: 'nav_msgs/Odometry'
});

var cmdVel = new ROSLIB.Topic({
  ros: ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/Twist'
});

var toggle_option = new ROSLIB.Topic({
  ros: ros,
  name: '/toggle',
  messageType: 'std_msgs/Int8'
});

var ui_params = new ROSLIB.Topic({
  ros: ros,
  name: '/ui_parameters_topic',
  messageType: 'std_msgs/String'
});

var ui_set_params = new ROSLIB.Topic({
  ros: ros,
  name: '/ui_set_parameters',
  messageType: 'std_msgs/String'
});

var ui_stop = new ROSLIB.Topic({
  ros: ros,
  name: '/manuel_stop',
  messageType: 'std_msgs/Int8'
});

var ui_vel = new ROSLIB.Topic({
  ros: ros,
  name: '/manuel_vel',
  messageType: 'std_msgs/Int8'
});



var twist = new ROSLIB.Message({
  linear: {
    x: 0.0,
    y: 0.0,
    z: 0.0
  },
  angular: {
    x: 0.0,
    y: 0.0,
    z: 0.0
  }
});

var ui_stop_data = new ROSLIB.Message({
  data: 1
});

var ui_vel_data = new ROSLIB.Message({
  data: 0
});

function manipulator_stop(){
  ui_vel_data.data = 1;
  ui_stop.publish(ui_vel_data) 
}

function manipulator_release(){
  ui_vel_data.data = 2;
  ui_stop.publish(ui_vel_data)
}

function move_forward() {
  
  twist.linear.x = parseFloat(document.getElementById("mr_ui_velocity").value);
  ui_vel_data.data = 1;
  ui_vel.publish(ui_vel_data);

  // twist.angular.z = 0.0;
  // cmdVel.publish(twist);
}

function move_backward() {
  ui_vel_data.data = -1;
  ui_vel.publish(ui_vel_data);

  twist.linear.x = -1 * parseFloat(document.getElementById("mr_ui_velocity").value);
  // twist.angular.z = 0.0;
  // cmdVel.publish(twist);
}

function move_stop() {
  ui_vel_data.data = 0;
  ui_vel.publish(ui_vel_data);
  // twist.linear.x = 0.0;
  // twist.angular.z = 0.0;
  // cmdVel.publish(twist);
}

odom_sub.subscribe(function (message) {
  document.getElementById("text_mr_postion").value = (message.pose.pose.position.x).toFixed(2);
  document.getElementById("text_mr_velocity").value = (message.twist.twist.linear.x).toFixed(2);
});

ui_params.subscribe(function (message)  {
  let text = (message.data).replace(/['"]+/g, '"');
  json_data = JSON.parse(text);
  // document.getElementById("mr_ui_rate").value = json_data.topic_rate;
  document.getElementById("mr_ui_velocity").value = json_data.robot_velocity;
  // document.getElementById("mr_ui_origin").value = json_data.sensor_origin_set;
  ui_params.unsubscribe();
  

});

var light_curtain = new ROSLIB.Topic({
  ros: ros,
  name: '/light_curtain',
  messageType: 'std_msgs/Bool'
});

light_curtain.subscribe(function (message)  {
  if (message.data == true){
    var emg_stop_data = new ROSLIB.Message({
      data: 1
    });
    
    ui_emg_stop.publish(emg_stop_data);
  }
});




var emg_continue_btn = document.getElementById("emergency_continue");

// When the user clicks the button, open the modal 
emg_continue_btn.onclick = function () {
  emgmodal.style.display = "none";
  var emg_stop_data = new ROSLIB.Message({
    data: 0
  });
  for (let i = 0; i < 5; i++) {
    ui_emg_stop.publish(emg_stop_data);
  }
}

// document.getElementById("mr_ui_rate").value = 10;
// document.getElementById("mr_ui_velocity").value = 0.1;
// document.getElementById("mr_ui_origin").value = 2.5;


var config_btn = document.getElementById("config_button");

config_btn.onclick = function () {
  // let rate = document.getElementById("mr_ui_rate").value;
  let vel = document.getElementById("mr_ui_velocity").value;
  // let origin = document.getElementById("mr_ui_origin").value;
  str_data = "{\"topic_rate\":50" + ",\"robot_velocity\":" + vel + ",\"sensor_origin_set\":2.5}";

  var config_data = new ROSLIB.Message({
    data: str_data
  });    
  ui_set_params.publish(config_data)
}

function toggle() {
  testBool = !testBool;
  if (testBool == false){
    testBool = 0;}
  else{
    testBool = 1;
  }
  var toggle_msg = new ROSLIB.Message({
    data: testBool
  });  
  toggle_option.publish(toggle_msg)
  if(testBool){
    document.getElementById("tasktracking").style.pointerEvents = "none";
    document.getElementById("tasktracking").style.opacity = 0.5;
    document.getElementById("mr_motion").style.pointerEvents = "auto";
    document.getElementById("mr_motion").style.opacity = 1.0;
  }
  else{
    document.getElementById("tasktracking").style.pointerEvents = "auto";
    document.getElementById("tasktracking").style.opacity = 1.0;
    document.getElementById("mr_motion").style.pointerEvents = "none";
    document.getElementById("mr_motion").style.opacity = 0.5;
  }
 }




/********************************/ 
// function connectRos() {

//   ros.on('connection', function () {
//      console.log("Connected to websocket server")
//      isConnected=true;
//  }) ;
//  ros.on('close', function (error) {
//    console.log("Close connecting to websocket server:", error);  
//    isConnected=false
// });
//  ros.on('error', function (error) {
//      console.log("Error connecting to websocket server:", error);  
//      isConnected=false
//  });
// }

//  $(document).ready(function() { 
//   connectRos();
//     setInterval(checkConnection,1000);
//  });

// function checkConnection(){
//   if(isConnected===false)        
//     connectRos();
// }