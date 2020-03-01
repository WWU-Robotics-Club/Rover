// initialize express router
const express = require('express');
const app = express();
const router = express.Router();

const path = __dirname + '/public/';
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
  res.sendFile(path + 'index.html');
});

// route requests to static files
app.use(express.static(path));
app.use('/', router);

app.listen(httpsPort, function () {
  console.log(`Example app listening on port ${httpsPort}`)
})


const wss = new WebSocket.Server({ port: wssPort });

wss.on('connection', function connection(ws, request, client) {
  console.log('New wss connection');
  const rosWs = new WebSocket(rosWsAddr);
  // relay messages from the client
  ws.on('message', function message(msg) {
    console.log(`Received message ${msg} from user`);
    rosWs.send(msg);
  });
  // relay messages from ros
  rosWs.on('message', function message(msg) {
    console.log(`Received message ${msg} from ROS`);
    ws.send(msg);
  });
});