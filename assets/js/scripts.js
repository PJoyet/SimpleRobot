let R = new Robot([0, 0, 0], [Math.PI / 2, 0, 0], [1, 1, 1])

var canvas = document.getElementById("game");
var ctx = canvas.getContext("2d");
canvas.width = canvas.getBoundingClientRect().width;
canvas.height = canvas.getBoundingClientRect().height;

var origin_x = canvas.width / 2;
var origin_y = canvas.height;
var dt = 0.1
var mouseX = R.getPosA().subset(math.index(0, 0));
var mouseY = R.getPosA().subset(math.index(0, 0));

document.addEventListener("mousemove", mouseMoveHandler, false);

function mouseMoveHandler(e) {
    var relativeX = e.clientX - canvas.offsetLeft;
    var relativeY = e.clientY - canvas.offsetTop;
    if (relativeX > 0 && relativeX < canvas.width && relativeY > 0 && relativeY < canvas.height) {
        mouseX = relativeX;
        mouseY = relativeY;
    }
}

function drawLine(robot, Coords, CoordEnd) {
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(Coords.subset(math.index(0, 0)), Coords.subset(math.index(1, 0)));
    for (let index = 1; index < robot.liaison.length; index++) {
        ctx.lineTo(Coords.subset(math.index(0, index)), Coords.subset(math.index(1, index)));
    }
    ctx.lineTo(CoordEnd[0], CoordEnd[1]);
    ctx.stroke();
}
console.log(R.getPosA().subset(math.index(0, 0)));

function drawRobot(robot, scale = 1) {
    var Coords = robot.getAllPosA();
    Coords = Coords.subset(math.index(math.range(0, 2), math.range(0, robot.liaison.length)));
    let origin = math.matrix([[origin_x], [origin_y]])
    Coords = math.multiply(Coords, scale);
    for (let index = 0; index < robot.liaison.length; index++) {
        var dataX = (Coords.subset(math.index(0, index)));
        var dataY = -(Coords.subset(math.index(1, index)));
        Coords.subset(math.index(0, index), dataX + origin_x);
        Coords.subset(math.index(1, index), dataY + origin_y);
    }
    var CoordEnd = [robot.getPosA().subset(math.index(0, 0)) * scale + origin_x, -robot.getPosA().subset(math.index(1, 0)) * scale + origin_y];


    return drawLine(robot, Coords, CoordEnd)
}

function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    var X = (mouseX - origin_x) / 100;
    var Y = (origin_y - mouseY) / 100;

    var angle = math.atan2(Y, X);
    var radius = 3
    if (Math.pow(X, 2) + Math.pow(Y, 2) - Math.pow(radius, 2) > 0) {
        X = radius * Math.cos(angle);
        Y = radius * Math.sin(angle);
    }

    R.simule([X, Y, 0], gain = 2, dt = 0.1);
    drawRobot(R, 100);
}
setInterval(draw, 100);


