/* global mat4 vec3 glMatrix vertexShaderCode fragmentShaderCode */
// Load our model.
const sess = new onnx.InferenceSession();
const loadingModelPromise = sess.loadModel("./modelInput50.onnx");
var gl;
var scalePerlinFactor;
var transPerlinFactor;
var anglePerlinFactor;
var scalePerlinFactor1;
var transPerlinFactor1;
var anglePerlinFactor1;

var brightPerlin1;
var brightPerlin2;
var weightPerlin1;
var weightPerlin2;
var modeSh;

// Function that is called on load.
function initDemo() {
  const canvas = document.getElementById('c');


  // Resize canvas HTML element to window size
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight-100;
  

  gl = canvas.getContext('webgl2');

  if (!gl) {
    // Some browsers have only experimental support.
    console.log('WebGL not supported. Using Experimental WebGL.');
    gl = canvas.getContext('experimental-webgl');
  }

  if (!gl) {
    alert('Your browser does not support WebGL!');
    return;
  }

  // Adjust viewport to window size
  gl.viewport(0, 0, canvas.width, canvas.heigh5);

  // Clear window in purple
  gl.clearColor(0.0, 0.0, 0.0, 1.0);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  // Create shaders
  // Prior to drawing, we need to compile the shaders.
  // This is due to OpenGL ES being a programmable shading interface

  const vertexShader = gl.createShader(gl.VERTEX_SHADER);
  const fragmentShader = gl.createShader(gl.FRAGMENT_SHADER);

  gl.shaderSource(vertexShader, vertexShaderCode);
  gl.shaderSource(fragmentShader, fragmentShaderCode);

  gl.compileShader(vertexShader);
  if (!gl.getShaderParameter(vertexShader, gl.COMPILE_STATUS)) {
    console.log(
      'Error compiling vertexShader',
      gl.getShaderInfoLog(vertexShader),
    );
    return;
  }

  gl.compileShader(fragmentShader);
  if (!gl.getShaderParameter(fragmentShader, gl.COMPILE_STATUS)) {
    console.log(
      'Error compiling fragmentShader',
      gl.getShaderInfoLog(fragmentShader),
    );
    return;
  }

  // Attach shaders to a GL program
  const program = gl.createProgram();
  gl.attachShader(program, vertexShader);
  gl.attachShader(program, fragmentShader);
  gl.linkProgram(program);
  gl.useProgram(program);
  // Additional checking if everything went fine
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    console.error(
      'Error linking program',
      gl.getProgramInfoLog(program),
    );
    return;
  }

  gl.validateProgram(program);
  if (!gl.getProgramParameter(program, gl.VALIDATE_STATUS)) {
    console.error(
      'Error validating program',
      gl.getProgramInfoLog(program),
    );
    return;
  }


  // Create screen corners in a buffer
  // As this application relies on the fragment fragment buffer,
  // we only need to tell OpenGL to draw the whole area and the
  // pixels are processed individually (and in parallel) in the
  // fragment shader
  const screenCorners = [
    //     X,        Y,
    /* */1.0, /* */1.0,
    /**/-1.0, /* */1.0,
    /**/-1.0, /**/-1.0,
    /* */1.0, /**/-1.0,
  ];

  const screenCornersVertexBufferObject = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, screenCornersVertexBufferObject);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(screenCorners), gl.STATIC_DRAW);

  // Vertices
  const vertexPositionAttribLocation =
    gl.getAttribLocation(program, 'vertexPosition');

  gl.vertexAttribPointer(
    vertexPositionAttribLocation, // index
    2, // size
    gl.FLOAT, // type
    gl.FALSE, // normalized
    0, // stride
    0, // offset
  );

  gl.enableVertexAttribArray(vertexPositionAttribLocation);

  const nearVertexBufferObject = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, nearVertexBufferObject);

  // Near vertices on attribute
  const nearPositionAttribLocation = gl.getAttribLocation(program, 'plotPosition');
  gl.vertexAttribPointer(
    nearPositionAttribLocation, // index
    3, // size
    gl.FLOAT, // type
    gl.FALSE, // normalized
    0, // stride
    0, // offset
  );

  gl.enableVertexAttribArray(nearPositionAttribLocation);

  // Bind program to WebGL
  gl.useProgram(program);

  // Set properties
  const cameraPositionLocation = gl.getUniformLocation(program, 'cameraPosition');
  const sphereCenterLocation1 = gl.getUniformLocation(program, 'sphere1');
  const sphereCenterLocation2 = gl.getUniformLocation(program, 'sphere2');

  const cubeCenterLocation1 = gl.getUniformLocation(program, 'cube1');
  const cubeCenterLocation2 = gl.getUniformLocation(program, 'cube2');

  const floorRadiusLocation = gl.getUniformLocation(program, 'floorRadius');
  const floorLocationLocation = gl.getUniformLocation(program, 'floorLocation');
  const planeDirectionLocation = gl.getUniformLocation(program, 'norPlane');

  scalePerlinFactor = gl.getUniformLocation(program, 'scalePerlin');
  transPerlinFactor = gl.getUniformLocation(program, 'transPerlin');
  anglePerlinFactor = gl.getUniformLocation(program, 'anglePerlin');
  scalePerlinFactor1 = gl.getUniformLocation(program, 'scalePerlin1');
  transPerlinFactor1 = gl.getUniformLocation(program, 'transPerlin1');
  anglePerlinFactor1 = gl.getUniformLocation(program, 'anglePerlin1');

  brightPerlin1 = gl.getUniformLocation(program, 'brightPerlin1');
  brightPerlin2 = gl.getUniformLocation(program, 'brightPerlin2');
  weightPerlin1 = gl.getUniformLocation(program, 'weightPerlin1');
  weightPerlin2 = gl.getUniformLocation(program, 'weightPerlin2');


  modeSh = gl.getUniformLocation(program, 'mode');

  var sphereCenters = [[5.0, 2.5, 0.0], [3.0, 2.5, -5.0]];
  var cubeCenters = [[-5.0, 2.5, 0.0], [-3.0, 2.5, -5.0]];
  
  var sphere1 = sphereCenters[0]
  var sphere2 = sphereCenters[1]
  var cube1 = cubeCenters[0]
  var cube2 = cubeCenters[1]

  gl.uniform3f(sphereCenterLocation1, sphere1[0], sphere1[1], sphere1[2]);
  gl.uniform3f(sphereCenterLocation2, sphere2[0], sphere2[1], sphere2[2]);

  gl.uniform3f(cubeCenterLocation1, cube1[0], cube1[1], cube1[2]);
  gl.uniform3f(cubeCenterLocation2, cube2[0], cube2[1], cube2[2]);

  
  gl.uniform1f(floorRadiusLocation, 20.0);
  gl.uniform3f(floorLocationLocation, 0.0, -1.0, 0.0);

  gl.uniform3f(scalePerlinFactor, 1.0,1.0,1.0);
  gl.uniform3f(transPerlinFactor, 1.0,1.0,1.0);
  gl.uniform3f(anglePerlinFactor, 0.0,0.0,0.0);

  var prevMod = 0;
  gl.uniform1i(modeSh, 0);

  const up = vec3.fromValues(0.0, 1.0, 0.0);
  const cameraTo = vec3.fromValues(0.0, 0.0, 0.0);
  const cameraInitialPosition = vec3.fromValues(0.0, 0.0, 10.0);
  const cameraPosition = new Float32Array(3);

  const cameraDirection = new Float32Array(3);
  const cameraUp = new Float32Array(3);
  const cameraLeft = new Float32Array(3);

  const nearCenter = new Float32Array(3);
  const nearTopLeft = new Float32Array(3);
  const nearBottomLeft = new Float32Array(3);
  const nearTopRight = new Float32Array(3);
  const nearBottomRight = new Float32Array(3);

  let ratio = canvas.width / canvas.height;

  const fpsValueElem = document.getElementById('fps_value');
  let fps = 0;

  function renderLoop() {
    var dirx = parseFloat(document.getElementById("dirx").value);
    var diry = parseFloat(document.getElementById("diry").value);
    var dirz = parseFloat(document.getElementById("dirz").value);
    if(dirx != NaN && diry != NaN && dirz != NaN ){
      gl.uniform3f(planeDirectionLocation, dirx, diry, dirz);
    }
    
    var perlinx = parseFloat(document.getElementById("perx").value);
    var perliny = parseFloat(document.getElementById("pery").value);
    var perlinz = parseFloat(document.getElementById("perz").value);
    if(perlinx != NaN && perliny != NaN && perlinz != NaN ){
      gl.uniform3f(scalePerlinFactor, perlinx, perliny, perlinz);
    }
    var perlinxTr = parseFloat(document.getElementById("perxTr").value);
    var perlinyTr = parseFloat(document.getElementById("peryTr").value);
    var perlinzTr = parseFloat(document.getElementById("perzTr").value);
    if(perlinxTr != NaN && perlinyTr != NaN && perlinzTr != NaN ){
      gl.uniform3f(transPerlinFactor, perlinxTr, perlinyTr, perlinzTr);
    }
    var perlinx = parseFloat(document.getElementById("perxrot").value)*Math.PI/180;
    var perliny = parseFloat(document.getElementById("peryrot").value)*Math.PI/180;
    var perlinz = parseFloat(document.getElementById("perzrot").value)*Math.PI/180;
    if(perlinx != NaN && perliny != NaN && perlinz != NaN ){
      if(perlinx >= Math.PI*2 ){
        document.getElementById("perxrot").value = 0.0
      }if(perliny >= Math.PI*2 ){
        document.getElementById("peryrot").value = 0.0
      }if(perlinz >= Math.PI*2 ){
        document.getElementById("perzrot").value = 0.0
      }
      gl.uniform3f(anglePerlinFactor, perlinx, perliny, perlinz);
    }

    var brR = parseFloat(document.getElementById("brR").value);
    var brG = parseFloat(document.getElementById("brG").value);
    var brB = parseFloat(document.getElementById("brB").value);
    if(brR != NaN && brG != NaN && brB != NaN ){
      gl.uniform3f(brightPerlin1, brR, brG, brB);
    }

    var wR = parseFloat(document.getElementById("wR").value);
    var wG = parseFloat(document.getElementById("wG").value);
    var wB = parseFloat(document.getElementById("wB").value);
    if(wR != NaN && wG != NaN && wB != NaN ){
      //console.log(wR, wG, wB)
      gl.uniform3f(weightPerlin1, wR, wG, wB);
    }

    var modecur = parseFloat(document.getElementById("mode").value);
    if(modecur != prevMod){
      if(modecur == 0 || modecur == 1 ){
        document.getElementById("perx").value = 1.0
        document.getElementById("pery").value = 1.0
        document.getElementById("perz").value = 1.0
        document.getElementById("brR").disabled = true
        document.getElementById("brG").disabled = true
        document.getElementById("brB").disabled = true
        document.getElementById("wR").disabled = true
        document.getElementById("wG").disabled = true
        document.getElementById("wB").disabled = true
      }else if(modecur == 2){
        document.getElementById("perx").value = 10.0
        document.getElementById("pery").value = 10.0
        document.getElementById("perz").value = 10.0
        document.getElementById("brR").disabled = true
        document.getElementById("brG").disabled = true
        document.getElementById("brB").disabled = true
        document.getElementById("wR").disabled = true
        document.getElementById("wG").disabled = true
        document.getElementById("wB").disabled = true
      }else if(modecur == 3){
        document.getElementById("perx").value = 60.0
        document.getElementById("pery").value = 20.0
        document.getElementById("perz").value = 60.0
        document.getElementById("brR").disabled = true
        document.getElementById("brG").disabled = true
        document.getElementById("brB").disabled = true
        document.getElementById("wR").disabled = true
        document.getElementById("wG").disabled = true
        document.getElementById("wB").disabled = true
      }else if(modecur == 4){
        document.getElementById("perx").value = 4.0
        document.getElementById("pery").value = 80.0
        document.getElementById("perz").value = 4.0
        document.getElementById("brR").disabled = true
        document.getElementById("brG").disabled = true
        document.getElementById("brB").disabled = true
        document.getElementById("wR").disabled = true
        document.getElementById("wG").disabled = true
        document.getElementById("wB").disabled = true
      }else if(modecur == 5){
        document.getElementById("brR").disabled = false
        document.getElementById("brG").disabled = false
        document.getElementById("brB").disabled = false
        document.getElementById("wR").disabled = false
        document.getElementById("wG").disabled = false
        document.getElementById("wB").disabled = false
      }
      gl.uniform1i(modeSh, modecur)
      prevMod = modecur
    }

    // resize canvas in case window size has changed
    if (canvas.width !== window.innerWidth
        || canvas.height !== window.innerHeight) {
      canvas.width = window.innerWidth;
      canvas.height = window.innerHeight;
      gl.viewport(0, 0, canvas.width, canvas.height);
      ratio = canvas.width / canvas.height;
    }

    //const angle = 2 * Math.PI * ((performance.now() / 1000.0) / 6.0);
    var cam = parseFloat(document.getElementById("cam").value) * (Math.PI/180);
    var angle;
    if(cam != NaN ){
      angle = cam;
    }else{
      angle = Math.Pi/4;
    }
    // Calc new camera position
    vec3.rotateY(cameraPosition, cameraInitialPosition, cameraTo, angle);

    
    //const angle = 2 * Math.PI * ((performance.now() / 1000.0) / 6.0);
    var camz = parseFloat(document.getElementById("camz").value) * (Math.PI/180);
    var anglez;
    if(camz != NaN ){
      anglez = camz;
    }else{
      anglez = Math.Pi/4;
    }
    // Calc new camera position
    vec3.rotateX(cameraPosition, cameraPosition, cameraTo, anglez);

    gl.uniform3f(
      cameraPositionLocation,
      cameraPosition[0],
      cameraPosition[1],
      cameraPosition[2],
    );

    // Calc new camera direction
    vec3.subtract(cameraDirection, cameraTo, cameraPosition);
    vec3.normalize(cameraDirection, cameraDirection);

    // Calc camera left vector
    vec3.cross(cameraLeft, up, cameraDirection);
    vec3.normalize(cameraLeft, cameraLeft);
    // Calc camera up vector
    vec3.cross(cameraUp, cameraDirection, cameraLeft);
    vec3.normalize(cameraUp, cameraUp);

    // Calc near plane center
    vec3.add(nearCenter, cameraPosition, cameraDirection);

    // Scale camera left to keep ratio
    vec3.scale(cameraLeft, cameraLeft, ratio);

    // Calc near corners
    // TopLeft
    vec3.add(nearTopLeft, nearCenter, cameraUp);
    vec3.add(nearTopLeft, nearTopLeft, cameraLeft);
    // BottomLeft
    vec3.subtract(nearBottomLeft, nearCenter, cameraUp);
    vec3.add(nearBottomLeft, nearBottomLeft, cameraLeft);
    // TopRight
    vec3.add(nearTopRight, nearCenter, cameraUp);
    vec3.subtract(nearTopRight, nearTopRight, cameraLeft);
    // BottomRight
    vec3.subtract(nearBottomRight, nearCenter, cameraUp);
    vec3.subtract(nearBottomRight, nearBottomRight, cameraLeft);

    const corners = new Float32Array(12);
    corners.set(nearTopRight, 0);
    corners.set(nearTopLeft, 3);
    corners.set(nearBottomLeft, 6);
    corners.set(nearBottomRight, 9);

    gl.bufferData(gl.ARRAY_BUFFER, corners, gl.STATIC_DRAW);
    gl.drawArrays(gl.TRIANGLE_FAN, 0, 4);

    fps += 1;

    requestAnimationFrame(renderLoop);
  }

  requestAnimationFrame(renderLoop);
  setInterval(() => {
    fpsValueElem.innerText = fps;
    fps = 0;
  }, 1000);
}

async function updatePredictions() {
  // Get the predictions for the canvas data.
  //const imgData = new Array(10*3*50*50).fill(0);
  //console.log([result.length,result[0].length,result[0][0].length,result[0][0][0].length])
  //console.log(result[0][0][0][0].length)
  const input = new onnx.Tensor(result, "float32", [1,3,width,height]);
  console.log(input)

  console.log(1)
  const outputMap = await sess.run([input]);
  console.log(2)
  const outputTensor = outputMap.values().next().value;
  console.log(3)
  const predictions = outputTensor.data;
  console.log(predictions) ;
  gl.uniform1i(modeSh, 10)

  var scalePerlinFactor;
var transPerlinFactor;
var anglePerlinFactor;

  gl.uniform3f(transPerlinFactor, 0,0,0);
  gl.uniform3f(transPerlinFactor1, 0,0,0);

  gl.uniform3f(scalePerlinFactor, predictions[0],predictions[2],predictions[2]);
  gl.uniform3f(scalePerlinFactor1, predictions[1],predictions[3],predictions[3]);
  gl.uniform3f(anglePerlinFactor, predictions[6],predictions[6],predictions[6]);
  gl.uniform3f(anglePerlinFactor1, predictions[7],predictions[7],predictions[7]);

  gl.uniform3f(brightPerlin1, predictions[7],predictions[9],predictions[11]);
  gl.uniform3f(brightPerlin2, predictions[8],predictions[10],predictions[12]);
  gl.uniform3f(weightPerlin1, predictions[13],predictions[15],predictions[17]);
  gl.uniform3f(weightPerlin2, predictions[14],predictions[18],predictions[18]);
  
}

function onImageChange(event) {
  const imageFile = URL.createObjectURL(event.target.files[0]);
  createImage(imageFile, convertImage);
}

function createImage(imageFile, callback) {
  const image = document.createElement('img');
  image.onload = () => callback(image);
  image.setAttribute('src', imageFile);
}

var result;
var height;
var width;

function convertImage(image) {
  const canvas = drawImageToCanvas(image);
  const ctx = canvas.getContext('2d');

  result = [];
  height = canvas.height;
  width = canvas.width;
  height = 50;
  width = 50;
  /*
  result[0].push([]);
  result[0].push([]);
  result[0].push([]);
  console.log(result[0])
  console.log(result[0][0])
  */
 console.log(result)
  for (let y = 0; y < height; y++) {
    //result[0][0].push([]);
    //result[0][1].push([]);
    //result[0][2].push([]);
    for (let x = 0; x < width; x++) {
      let data = ctx.getImageData(x, y, 1, 1).data;
      //console.log('herrrr')
      //result[0][0][y].push([]);
      //result[0][1][y].push([]);
      //result[0][2][y].push([]);
      result.push(data[0] / 255);
      result.push(data[1] / 255);
      result.push(data[2] / 255);
    }
  }

  console.log(result)

  updatePredictions()

}

function drawImageToCanvas(image) {
  const canvas = document.createElement('canvas');
  canvas.width = image.width;
  canvas.height = image.height;
  canvas.getContext('2d').drawImage(image, 0, 0, image.width, image.height);
  return canvas;
}

function convertArray(array) {
  return JSON.stringify(array).replace(/\[/g, '{').replace(/\]/g, '}');
}


