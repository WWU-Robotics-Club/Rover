// initialize express router
const express = require('express');
const app = express();
const router = express.Router();

const staticPath = __dirname + '/public/';
const httpsPort = 8080;

const WebSocket = require('ws');
const wssPort = 8081;
const rosWsAddr = 'ws://ros-master:9090';

// success middleware logger
router.use(function (req,res,next) {
  console.log('/' + req.method);
  next();
});

// route GET requests to / to index.html
router.get('/',function(req,res){
  res.sendFile(staticPath + 'index.html');
});

// route requests to static files
app.use(express.static(staticPath));
app.use('/', router);

app.listen(httpsPort, function () {
  console.log(`Example app listening on port ${httpsPort}`)
})

// websocket server that users can connect to
const wss = new WebSocket.Server({ port: wssPort });
var rosWs;
var connectRosWs = function() {
  console.log('Connecting to ROS websocket');
  rosWs = new WebSocket(rosWsAddr);
  rosWs.on('error', function(error) {
    console.log('ROS websocket error: ' + error.message);
  });
  rosWs.on('open', function() {
    console.log('ROS websocket opened');
  });
  rosWs.on('close', function() {
    console.log('ROS websocket closed');
    setTimeout(connectRosWs, 5000); // attempt to reconnect every 5 seconds
  });
}
connectRosWs();

wss.on('connection', function connection(ws, request, client) {
  console.log('New wss connection');
  //const rosWs = new WebSocket(rosWsAddr);
  // relay messages from the client to ROS
  ws.on('message', function message(msg) {
    console.log(`Received message ${msg} from user`);
    rosWs.send(msg);
  });
  // Relay messages from ROS to the client.
  // With rosWs being a global, all clients will get the same info.
  // So if a second client subscribes to a topic, those messages will also be sent to the first.
  // Not ideal but seems a bit more stable. Needs more work
  rosWs.on('message', function message(msg) {
    console.log(`Received message ${msg} from ROS`);
    ws.send(msg);
  });
});