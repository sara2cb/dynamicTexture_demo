// Load our model.
const sess = new onnx.InferenceSession();
const loadingModelPromise = sess.loadModel("./myDesignModel_v.onnx");

//Define global variables
var gl;
var scalePerlinFactor;
var transPerlinFactor;
var anglePerlinFactor;
var scalePerlinFactor1;
var transPerlinFactor1;
var anglePerlinFactor1;
var scalePerlinFactor2;
var transPerlinFactor2;
var anglePerlinFactor2;

var brightPerlin1;
var brightPerlin2;
var brightPerlin3;

var weightPerlin1;
var weightPerlin2;
var weightPerlin3;
var realImageOn = false;
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

  const lighPosLocation = gl.getUniformLocation(program, 'lightPos');

  //Set Perlin parameters
  scalePerlinFactor = gl.getUniformLocation(program, 'scalePerlin');
  transPerlinFactor = gl.getUniformLocation(program, 'transPerlin');
  anglePerlinFactor = gl.getUniformLocation(program, 'anglePerlin');
  scalePerlinFactor1 = gl.getUniformLocation(program, 'scalePerlin1');
  transPerlinFactor1 = gl.getUniformLocation(program, 'transPerlin1');
  anglePerlinFactor1 = gl.getUniformLocation(program, 'anglePerlin1');
  scalePerlinFactor2 = gl.getUniformLocation(program, 'scalePerlin2');
  transPerlinFactor2 = gl.getUniformLocation(program, 'transPerlin2');
  anglePerlinFactor2 = gl.getUniformLocation(program, 'anglePerlin2');

  brightPerlin1 = gl.getUniformLocation(program, 'brightPerlin1');
  brightPerlin2 = gl.getUniformLocation(program, 'brightPerlin2');
  brightPerlin3 = gl.getUniformLocation(program, 'brightPerlin3');

  weightPerlin1 = gl.getUniformLocation(program, 'weightPerlin1');
  weightPerlin2 = gl.getUniformLocation(program, 'weightPerlin2');
  weightPerlin3 = gl.getUniformLocation(program, 'weightPerlin3');

  //Set other input fields
  modeSh = gl.getUniformLocation(program, 'mode');
  reflectionOn = gl.getUniformLocation(program, 'reflectingOn');
  gridOn = gl.getUniformLocation(program, 'gridOn');

  //Set geometry locations
  var sphere1 = [5.0, 2, 0.0]
  var cube1 = [-5.0, 2, 0.0]

  gl.uniform3f(sphereCenterLocation1, sphere1[0], sphere1[1], sphere1[2]);
  //gl.uniform3f(sphereCenterLocation2, sphere2[0], sphere2[1], sphere2[2]);

  gl.uniform3f(cubeCenterLocation1, cube1[0], cube1[1], cube1[2]);
  //gl.uniform3f(cubeCenterLocation2, cube2[0], cube2[1], cube2[2]);
  
  gl.uniform1f(floorRadiusLocation, 30.0);
  gl.uniform3f(floorLocationLocation, 0.0, -1.0, 0.0);

  var lightPos =[0, 9, 10]
  gl.uniform3f(lighPosLocation, lightPos[0], lightPos[1], lightPos[2]);

  gl.uniform3f(scalePerlinFactor, 1.0,1.0,1.0);
  gl.uniform3f(transPerlinFactor, 1.0,1.0,1.0);
  gl.uniform3f(anglePerlinFactor, 0.0,0.0,0.0);

  //Set variables for modes
  var prevMod = 0;
  gl.uniform1i(modeSh, 0);
  var prevReflection = true;
  gl.uniform1i(reflectionOn, true);
  var prevGrid = true;
  gl.uniform1i(gridOn, true);

  let ratio = canvas.width / canvas.height;
  var prevrealImageOn = false

  //Camera parameters
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

  function renderLoop() {
    prevrealImageOn = realImageOn

    //Change modes
    var modecur = parseFloat(document.getElementById("mode").value);
    if(modecur != prevMod || prevrealImageOn == true){
      //Perlin grid and interpolation
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
        document.getElementById("perx").disabled = false
        document.getElementById("pery").disabled = false
        document.getElementById("perz").disabled = false
        document.getElementById("perxrot").disabled = false
        document.getElementById("peryrot").disabled = false
        document.getElementById("perzrot").disabled = false
        realImageOn == false 
      }
      //Marble
      else if(modecur == 2){
        document.getElementById("perx").value = 10.0
        document.getElementById("pery").value = 10.0
        document.getElementById("perz").value = 10.0
        document.getElementById("brR").disabled = true
        document.getElementById("brG").disabled = true
        document.getElementById("brB").disabled = true
        document.getElementById("wR").disabled = true
        document.getElementById("wG").disabled = true
        document.getElementById("wB").disabled = true

        document.getElementById("perx").disabled = false
        document.getElementById("pery").disabled = false
        document.getElementById("perz").disabled = false
        document.getElementById("perxrot").disabled = false
        document.getElementById("peryrot").disabled = false
        document.getElementById("perzrot").disabled = false

        realImageOn == false 
      }
      //Grass
      else if(modecur == 3){
        document.getElementById("perx").value = 60.0
        document.getElementById("pery").value = 20.0
        document.getElementById("perz").value = 60.0
        document.getElementById("brR").disabled = true
        document.getElementById("brG").disabled = true
        document.getElementById("brB").disabled = true
        document.getElementById("wR").disabled = true
        document.getElementById("wG").disabled = true
        document.getElementById("wB").disabled = true

        document.getElementById("perx").disabled = false
        document.getElementById("pery").disabled = false
        document.getElementById("perz").disabled = false
        document.getElementById("perxrot").disabled = false
        document.getElementById("peryrot").disabled = false
        document.getElementById("perzrot").disabled = false

        realImageOn == false 
      }
      //Wood
      else if(modecur == 4){
        document.getElementById("perx").value = 4.0
        document.getElementById("pery").value = 80.0
        document.getElementById("perz").value = 4.0
        document.getElementById("brR").disabled = true
        document.getElementById("brG").disabled = true
        document.getElementById("brB").disabled = true
        document.getElementById("wR").disabled = true
        document.getElementById("wG").disabled = true
        document.getElementById("wB").disabled = true

        document.getElementById("perx").disabled = false
        document.getElementById("pery").disabled = false
        document.getElementById("perz").disabled = false
        document.getElementById("perxrot").disabled = false
        document.getElementById("peryrot").disabled = false
        document.getElementById("perzrot").disabled = false

        realImageOn == false 
      }else if(modecur == 5){
        document.getElementById("brR").disabled = false
        document.getElementById("brG").disabled = false
        document.getElementById("brB").disabled = false
        document.getElementById("wR").disabled = false
        document.getElementById("wG").disabled = false
        document.getElementById("wB").disabled = false

        document.getElementById("perx").disabled = false
        document.getElementById("pery").disabled = false
        document.getElementById("perz").disabled = false
        document.getElementById("perxrot").disabled = false
        document.getElementById("peryrot").disabled = false
        document.getElementById("perzrot").disabled = false

        realImageOn == false 
      }
      gl.uniform1i(modeSh, modecur)
      prevMod = modecur

      //Real image when using a model
      if(prevrealImageOn == true ){
        document.getElementById("perx").disabled = true
        document.getElementById("pery").disabled = true
        document.getElementById("perz").disabled = true
        document.getElementById("perxrot").disabled = true
        document.getElementById("peryrot").disabled = true
        document.getElementById("perzrot").disabled = true

        document.getElementById("brR").disabled = true
        document.getElementById("brG").disabled = true
        document.getElementById("brB").disabled = true
        document.getElementById("wR").disabled = true
        document.getElementById("wG").disabled = true
        document.getElementById("wB").disabled = true

        realImageOn = false
        gl.uniform1i(modeSh, 10)
      }
    }


    //Reflection on
    var reflection = document.getElementById("reflectionCheck").checked
    if(reflection != prevReflection){
      gl.uniform1i(reflectionOn, reflection)
    }
    prevReflection = reflection


    //Plane direction
    var dirx = parseFloat(document.getElementById("dirx").value);
    var diry = parseFloat(document.getElementById("diry").value);
    var dirz = parseFloat(document.getElementById("dirz").value);
    if(dirx != NaN && diry != NaN && dirz != NaN ){
      gl.uniform3f(planeDirectionLocation, dirx, diry, dirz);
    }

    //Perlin scale parameters
    if(!document.getElementById("perx").disabled &&  !document.getElementById("pery").disabled && !document.getElementById("perz").disabled){
      var perlinx = parseFloat(document.getElementById("perx").value);
      var perliny = parseFloat(document.getElementById("pery").value);
      var perlinz = parseFloat(document.getElementById("perz").value);
      if(perlinx != NaN && perliny != NaN && perlinz != NaN){
        gl.uniform3f(scalePerlinFactor, perlinx, perliny, perlinz);
      }
    }

    //Perlin translation parameters
    var perlinxTr = parseFloat(document.getElementById("perxTr").value);
    var perlinyTr = parseFloat(document.getElementById("peryTr").value);
    var perlinzTr = parseFloat(document.getElementById("perzTr").value);
    if(perlinxTr != NaN && perlinyTr != NaN && perlinzTr != NaN ){
      gl.uniform3f(transPerlinFactor, perlinxTr, perlinyTr, perlinzTr);
    }

    //Perlin rotation parameters
    if(!document.getElementById("perxrot").disabled &&  !document.getElementById("peryrot").disabled && !document.getElementById("perzrot").disabled){
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
    }

    //Perlin brightness RGB parameters
    if(!document.getElementById("brR").disabled &&  !document.getElementById("brG").disabled && !document.getElementById("brB").disabled){
      var brR = parseFloat(document.getElementById("brR").value);
      var brG = parseFloat(document.getElementById("brG").value);
      var brB = parseFloat(document.getElementById("brB").value);
      if(brR != NaN && brG != NaN && brB != NaN){
        gl.uniform3f(brightPerlin1, brR, brG, brB);
      }
    }

    //Perlin weight RGB parameters
    if(!document.getElementById("wR").disabled &&  !document.getElementById("wG").disabled && !document.getElementById("wB").disabled){
      var wR = parseFloat(document.getElementById("wR").value);
      var wG = parseFloat(document.getElementById("wG").value);
      var wB = parseFloat(document.getElementById("wB").value);
      if(wR != NaN && wG != NaN && wB != NaN ){
        gl.uniform3f(weightPerlin1, wR, wG, wB);
      }
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


    requestAnimationFrame(renderLoop);
  }

  requestAnimationFrame(renderLoop);
}

async function updatePredictions() {
  // Get the predictions for the canvas data.
  //Create input image format
  const input = new onnx.Tensor(result, "float32", [1,3,width,height]);

  //Run model
  var start = new Date().getTime();
  const outputMap = await sess.run([input]);
  var end = new Date().getTime();
  var time = end - start;
  document.getElementById("inferTime").innerHTML = "Time of inference: " + time + "ms"

  //Get output of the model
  const outputTensor = outputMap.values()
  var predictionsRGB = outputTensor.next().value.data;
  var predictionsScale = outputTensor.next().value.data;
  var predictionsAngle = outputTensor.next().value.data;
  predictionsScale = predictionsScale.map(x => x * 2);

  //Set the Erlin parameters
  gl.uniform3f(transPerlinFactor, 0,0,0);
  gl.uniform3f(transPerlinFactor1, 0,0,0);
  
  gl.uniform3f(scalePerlinFactor, predictionsScale[0],predictionsScale[3],predictionsScale[0]);
  gl.uniform3f(scalePerlinFactor1, predictionsScale[1],predictionsScale[4],predictionsScale[1]);
  gl.uniform3f(scalePerlinFactor2, predictionsScale[2],predictionsScale[5],predictionsScale[2]);

  gl.uniform3f(anglePerlinFactor, predictionsAngle[0],predictionsAngle[0],predictionsAngle[0]);
  gl.uniform3f(anglePerlinFactor1, predictionsAngle[1],predictionsAngle[1],predictionsAngle[1]);
  gl.uniform3f(anglePerlinFactor2, predictionsAngle[0],predictionsAngle[0],predictionsAngle[0]);

  gl.uniform3f(brightPerlin1, predictionsRGB[0],predictionsRGB[3],predictionsRGB[6]);
  gl.uniform3f(brightPerlin2, predictionsRGB[1],predictionsRGB[4],predictionsRGB[7]);
  gl.uniform3f(brightPerlin3, predictionsRGB[2],predictionsRGB[5],predictionsRGB[8]);

  gl.uniform3f(weightPerlin1, predictionsRGB[9],predictionsRGB[12],predictionsRGB[15]);
  gl.uniform3f(weightPerlin2, predictionsRGB[10],predictionsRGB[13],predictionsRGB[16]);
  gl.uniform3f(weightPerlin3, predictionsRGB[11],predictionsRGB[14],predictionsRGB[17]);

  //Set mode
  gl.uniform1i(modeSh, 10)
  realImageOn = true
}

var result;
var height;
var width;

function convertImage(image) {
  //Get input image
  const canvas = drawImageToCanvas(image);
  const ctx = canvas.getContext('2d');

  result = [];
  height = 50;
  width = 50;

  //Save the image in format RGB
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      let data = ctx.getImageData(x, y, 1, 1).data;
      result.push(data[0] / 255);
    }
  }
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      let data = ctx.getImageData(x, y, 1, 1).data;
      result.push(data[1] / 255);
    }
  }
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      let data = ctx.getImageData(x, y, 1, 1).data;
      result.push(data[2] / 255);
    }
  }

  //Run model
  updatePredictions()
}

function drawImageToCanvas(image) {
  const canvas = document.createElement('canvas');
  canvas.width = image.width;
  canvas.height = image.height;
  canvas.getContext('2d').drawImage(image, 0, 0, image.width, image.height);
  return canvas;
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



