let W = new Robot2([0, 0, 0], [Math.PI / 2, 0, 0], [1, 1, 1],["revolute","revolute","revolute"])
var canvas = document.getElementById("game2");

// console.log(R.Jacobienne());


var ctx2 = canvas.getContext("2d");
canvas.width = canvas.getBoundingClientRect().width;
canvas.height = canvas.getBoundingClientRect().height;

var origin_x = canvas.width / 2;
var origin_y = canvas.height;
console.log(canvas.width);
var dt = 0.1
var mouseX = W.getPosA().subset(math.index(0, 0));
var mouseY = W.getPosA().subset(math.index(0, 0));

var touch = false;

document.addEventListener("mousemove", mouseMoveHandler, false);
document.addEventListener('touchstart', handleStart, false);

function mouseMoveHandler(e) {
    var relativeX = e.clientX - canvas.getBoundingClientRect().left;
    var relativeY = e.clientY - canvas.getBoundingClientRect().top;
    if (relativeX > 0 && relativeX < canvas.width && relativeY > 0 && relativeY < canvas.height) {
        mouseX = relativeX;
        mouseY = relativeY;
    }

}

function handleStart(e) {
    var relativeX = e.touches[0].clientX - canvas.offsetLeft;
    var relativeY = e.touches[0].clientY- canvas.offsetTop;
    if (relativeX > 0 && relativeX < canvas.width && relativeY > 0 && relativeY < canvas.height) {
        mouseX = relativeX;
        mouseY = relativeY;
    }
    touch=true
}


function drawX(x=mouseX, y=mouseY,size = 10) {
    ctx2.save();
    ctx2.beginPath();

    ctx2.moveTo(x - size, y - size);
    ctx2.lineTo(x + size, y + size);

    ctx2.moveTo(x + size, y - size);
    ctx2.lineTo(x - size, y + size);
    ctx2.strokeStyle = 'green';
    ctx2.lineWidth = 6;
    ctx2.stroke();
    ctx2.restore();
}

function drawLine(robot, Coords, CoordEnd) {
if (touch==true) {
drawX();
}

    
    ctx2.beginPath();
    ctx2.lineWidth = 4;
    ctx.moveTo(Coords.subset(math.index(0, 0)), Coords.subset(math.index(1, 0)));
    for (let index = 1; index < robot.liaison.length; index++) {
        ctx2.lineTo(Coords.subset(math.index(0, index)), Coords.subset(math.index(1, index)));
    }
    ctx2.lineTo(CoordEnd[0], CoordEnd[1]);
    ctx2.stroke();

    ctx2.beginPath();
    ctx2.arc(origin_x, origin_y, 10, 0, Math.PI * 2, false);
    ctx2.fillStyle = "black";
    ctx2.fill();
    ctx2.closePath();

    for (let index = 1; index < robot.liaison.length; index++) {
        ctx2.beginPath();
        ctx2.arc(Coords.subset(math.index(0, index)), Coords.subset(math.index(1, index)), 5, 0, Math.PI * 2, false);
        ctx2.fillStyle = "black";
        ctx2.fill();
        ctx2.closePath();
    }

    var dim = 12;
    var point = [[0, dim], [dim, dim],
    [dim, dim - dim / 3], [dim / 2, dim - dim / 3],
    [dim / 2, dim / 3], [dim, dim / 3],
    [dim, 0], [0, 0]];

}

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
    ctx2.clearRect(0, 0, canvas.width, canvas.height);
    var X = (mouseX - origin_x) / 100;
    var Y = (origin_y - mouseY) / 100;

    var angle = math.atan2(Y, X);
    var radius = 3
    if (Math.pow(X, 2) + Math.pow(Y, 2) - Math.pow(radius, 2) > 0) {
        X = radius * Math.cos(angle);
        Y = radius * Math.sin(angle);
    }

    W.simule([X, Y, 0], gain = 2, dt = 0.1);
    drawRobot(R, 100);
}
setInterval(draw, 100);



