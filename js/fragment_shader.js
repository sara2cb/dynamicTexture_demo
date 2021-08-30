const fragmentShaderCode = `#version 300 es

#define PI 0.31415926538
#define PHI 0.161803398874989484820459
#define THETA 0.078539816339
#define SQ2 14142.1356237309504880169
#define noCubes 1
#define noSpheres 1

precision highp float;

out vec4 fragColor;

in vec3 nearPosition;

uniform vec3 cameraPosition;

uniform vec3 sphere1;
uniform vec3 sphere2;
vec3 sphereCenter[noSpheres] ;

uniform vec3 cube1;
uniform vec3 cube2;
vec3 cubeCenter[noCubes] ;


uniform vec3 norPlane;
uniform vec3 scalePerlin;
uniform vec3 transPerlin;
uniform vec3 anglePerlin;
uniform vec3 scalePerlin1;
uniform vec3 transPerlin1;
uniform vec3 anglePerlin1;
uniform vec3 scalePerlin2;
uniform vec3 transPerlin2;
uniform vec3 anglePerlin2;

uniform vec3 brightPerlin1;
uniform vec3 weightPerlin1;
uniform vec3 brightPerlin2;
uniform vec3 weightPerlin2;
uniform vec3 brightPerlin3;
uniform vec3 weightPerlin3;

uniform vec3 floorLocation;
uniform float floorRadius;

uniform int mode;
uniform bool reflectingOn;
uniform bool gridOn;

vec3 A[noCubes], B[noCubes], C[noCubes], D[noCubes], E[noCubes], F[noCubes], G[noCubes], H[noCubes];


float cs;

vec3 lightPosition;
  
vec3 Ii;
float ka, kd, ks;
float n;

vec3 rotate(vec3 origin, vec3 point, vec3 angle){
    /*
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    */

    float qx = origin[0] + cos(angle[2]) * (point[0] - origin[0]) - sin(angle[2]) * (point[1] - origin[1]);
    float qy = origin[1] + sin(angle[2]) * (point[0] - origin[0]) + cos(angle[2]) * (point[1] - origin[1]);
    vec3 newPos = vec3(qx, qy, point[2]);

    qy = origin[1] + cos(angle[0]) * (newPos[1] - origin[1]) - sin(angle[0]) * (newPos[2] - origin[2]);
    float qz = origin[2] + sin(angle[0]) * (newPos[1] - origin[1]) + cos(angle[0]) * (newPos[2] - origin[2]);
    newPos = vec3(newPos[0], qy, qz);

    qz = origin[2] + cos(angle[1]) * (newPos[2] - origin[2]) - sin(angle[1]) * (newPos[0] - origin[0]);
    qx = origin[0] + sin(angle[1]) * (newPos[2] - origin[2]) + cos(angle[1]) * (newPos[0] - origin[0]);
    newPos = vec3(qx, newPos[1], qz);

    return newPos;
  }

int get_neighbour_offset(int i, int j) {

  return (i >> j) & 1;
}

const int dim = 3;

float get_nearest_noise(vec3 position, float seed, vec3 trans, vec3 scale, vec3 rotation) {

  float d = 0.0;

  position = (rotate(vec3(0.0), position, rotation)+transPerlin)*scalePerlin;

  for (int index_dim = 0; index_dim < dim; index_dim++) {
      float a = PHI;

      if (index_dim == 1) {
          a = PI;
      }

      if (index_dim == 2) {
          a = THETA;
      }

      float p = (position[index_dim]);
      float p_floor = floor(p);
      float b = p_floor * (seed + PHI) - a;
      d += b * b;
  }

  float s = sqrt(d + 1.0e-8);
  float t = tan(s) * SQ2;
  float noise = t - floor(t);

  return noise;
}

float get_nearest_noise(vec3 position, float seed) {

  float d = 0.0;

  for (int index_dim = 0; index_dim < dim; index_dim++) {
      float a = PHI;

      if (index_dim == 1) {
          a = PI;
      }

      if (index_dim == 2) {
          a = THETA;
      }

      float p = (position[index_dim]);
      float p_floor = floor(p);
      float b = p_floor * (seed + PHI) - a;
      d += b * b;
  }

  float s = sqrt(d + 1.0e-8);
  float t = tan(s) * SQ2;
  float noise = t - floor(t);

  return noise;
}

const int perm = int(pow(2.0,float(dim)));

float  trilinearNoise(vec3 position, float seed, vec3 trans, vec3 scale, vec3 rotation){
  float noise = 0.0;

  position = (rotate(vec3(0.0), position, rotation)+transPerlin)*scalePerlin;

  // calculate bilinear noise
  // reference to bilinear interpolation:
  // https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/interpolation/bilinear-filtering
  // https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/interpolation/trilinear-interpolation
  for(int j = 0; j < perm; j++) {

      float weight = 1.0;
      
      // calculate weights for interpolation
      for (int i = 0; i < dim; i++) {
          float lambda = (position[i] - 1.0) - floor(position[i] - 1.0);
          int offset = get_neighbour_offset(j,i);
 
          if (offset == 0) {
              weight = weight * (1.0 - lambda);
          }
          else {
              weight = weight * lambda;
          }
      }

      for(int p = 0; p < dim; p++) {
          int offset = get_neighbour_offset(j,p);
          position[p] += float(offset) - 1.0;
      }

      float nearest_noise = get_nearest_noise(position, seed);

      noise = noise + weight * nearest_noise ;

      for(int q = 0; q < dim; q++) {
          int offset = get_neighbour_offset(j,q);
          position[q] -=  float(offset) - 1.0;
      }
  }

  return noise;
}

bool intersectSphere(vec3 origin, vec3 rayDirection, vec3 sphereCenter, out vec3 intersection,
                     out float dist,
                     out vec3 V, out vec3 N, out vec3 L) {
  vec3 rayToSphere = sphereCenter - origin;
  float b = dot(rayDirection, rayToSphere);
  if (b < 0.0) return false;
  float d = sqrt( dot(rayToSphere, rayToSphere) - b * b);
  if (d > 2.5) return false;

  

  dist = b - sqrt(2.5*2.5 - d*d);
  if (dist < 0.0) return false;

  intersection = origin + rayDirection * dist;
  V = -rayDirection;
  N = normalize(intersection - sphereCenter);
  L = normalize(lightPosition - intersection);

  return true;
}

bool intersectFloor(vec3 origin, vec3 rayDirection, out vec3 intersection,
                    out float dist,
                    out vec3 V, out vec3 N, out vec3 L,
                    out vec3 color) {
  dist = (floorLocation[1] - origin.y) / rayDirection.y;
  if (dist < 0.0) return false;

  float x = origin.x + rayDirection.x * dist  + floorLocation[0];
  float z = origin.z + rayDirection.z * dist + floorLocation[2];
  if (x*x+ z*z > floorRadius*floorRadius  ) return false;

  if(gridOn){
    if(mod(x, 1.5) < 0.07 || mod(z, 1.5) < 0.07) color = vec3(1.0, 1.0, 1.0);
    else color = vec3(0.5, 0.5, 0.5);
  }
  else{
    if (x < 0.0 && z < 0.0) color = vec3(1.0, 1.0, 0.0);
    else if (x < 0.0 && z > 0.0) color = vec3(1.0, 0.0, 1.0);
    else if (x > 0.0 && z < 0.0) color = vec3(0.0, 0.5, 0.5);
    else if (x > 0.0 && z > 0.0) color = vec3(0.5, 0.5, 0.5);
  }

  intersection = vec3(x, floorLocation[1], z);
  V = -rayDirection;
  N = vec3(0.0, 1.0, 0.0);
  L = normalize(lightPosition - intersection);

  return true;
}

bool intersectPlane(vec3 origin, vec3 rayDirection, vec3 origPlane, vec3 norPlane, out vec3 intersection,
  out float dist,
  out vec3 V, out vec3 N, out vec3 L) {
  
  float denom = dot(rayDirection, normalize(norPlane));
  if(abs(denom) > 0.00001){
    vec3 subs = origPlane - origin;
    float t = dot(subs, normalize(norPlane)) / denom;
    dist = t;
    if(t<0.0){
      return false;
    }
    intersection = origin + t * rayDirection;
    V = -rayDirection;
    N = norPlane;
    L = normalize(lightPosition - intersection);

    return true;
  }
  return false;
}

bool intersectTriangle(vec3 origin, vec3 rayDirection, out vec3 intersection,
                       out float dist,
                       vec3 A, vec3 B, vec3 C,
                       out vec3 V, out vec3 N, out vec3 L) {
    
    // adapted from Moller-Trumbore intersection algorithm pseudocode on wikipedia
    vec3 AB, AC; // Edge1, Edge2
    vec3 P, Q, T;
    float det, inv_det, u, v;
    float t;
    
    // vectors for edges sharing V1
    AB = B - A;
    AC = C - A;

    // begin calculating determinant - also used to calculate u param
    P = cross(rayDirection, AC);

    // if determinant is near zero, ray lies in plane of triangle
    det = dot(AB, P);
    // culling
    if (det < 0.0) return false;
    inv_det = 1.0 / det;

    // calculate distance from A to ray origin
    T = origin - A;

    // calculate u parameter and test bound
    u = dot(T, P) * inv_det;
    // the intersection lies outside of the triangle
    if (u < 0.0 || u > 1.0) return false;

    // prepare to test v parameter
    Q = cross(T, AB);

    // calculate v param and test bound
    v = dot(rayDirection, Q) * inv_det;

    // the intersection is outside the triangle?
    if (v < 0.0 || (u + v) > 1.0) return false;

    t = dot(AC, Q) * inv_det;

    if (t < 0.0) return false;

    dist = t;
    intersection = origin + t * rayDirection;
    V = -rayDirection;
    N = normalize(cross(AB, AC));
    L = normalize(lightPosition - intersection);

    return true;
}

bool intersectCube(vec3 origin, vec3 rayDirection, int cubeI, out vec3 intersection,
                   out float dist,
                   out vec3 V, out vec3 N, out vec3 L) {

  if (intersectTriangle(origin, rayDirection, intersection, dist, A[cubeI], C[cubeI], B[cubeI], V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, A[cubeI], D[cubeI], C[cubeI], V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, A[cubeI], F[cubeI], E[cubeI], V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, A[cubeI], B[cubeI], F[cubeI], V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, A[cubeI], E[cubeI], H[cubeI], V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, A[cubeI], H[cubeI], D[cubeI], V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, G[cubeI], E[cubeI], F[cubeI], V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, G[cubeI], H[cubeI], E[cubeI], V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, G[cubeI], F[cubeI], B[cubeI], V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, G[cubeI], B[cubeI], C[cubeI], V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, G[cubeI], C[cubeI], D[cubeI], V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, G[cubeI], D[cubeI], H[cubeI], V, N, L))
    return true;

  return false;
}

bool insideCube(vec3 origin, vec3 cubeCenter) {

  vec3 diff = origin - cubeCenter;
  if(abs(diff[0]) > (5.0/2.0) || abs(diff[1]) > (5.0/2.0) || abs(diff[2]) > (5.0/2.0)){
    return false;
  }
  return true;
}

bool insideSphere(vec3 origin, vec3 sphereCenter) {

  vec3 diff = origin - sphereCenter;
  if(length(diff) > 2.5){
    return false;
  }
  return true;
}
bool frontPlane(vec3 origin, vec3 origPlane, vec3 norPlane) {
  vec3 subs = origPlane - origin;
  float t = dot(subs, normalize(norPlane));
  return (t<0.0);
}

vec3 colorAt(vec3 color, vec3 V, vec3 N, vec3 L, vec3 R) {
  vec3 rColor = 
    ka * color + (kd * dot(L, N) + ks * pow(dot(V, R), n)) * Ii;

  return rColor;
}

bool intersectSomethingGoingToLight(vec3 origin, vec3 N_aux) {
  float distToLight = length(lightPosition - origin);
  vec3 rayDirection = normalize(lightPosition - origin);

  if (dot(N_aux, rayDirection) < 0.0) return false;

  vec3 intersection;
  float dist;
  vec3 V, N, L;

  for(int i; i < noCubes; i++){
    bool interCube = intersectCube(origin, rayDirection, i, intersection, dist, V,N,L) && \
                !frontPlane(intersection, cubeCenter[i], norPlane);
    if (interCube){
      if (dist < distToLight) return true;
    }
  }

  for(int i; i < noSpheres; i++){
    bool interSphere = intersectSphere(origin, rayDirection, sphereCenter[i], intersection, dist, V,N,L) && \
              !frontPlane(intersection, sphereCenter[i], norPlane);
    if (interSphere){
      if (dist < distToLight) return true;
    }
  }
  

  return false;
}

bool intersectSomething(vec3 origin, vec3 rayDirection, out vec3 intersection,
                        out vec3 N, out vec3 color, out vec3 reflectedColor,
                        out bool shadow, out bool isFloor) {
  float dist[1+noSpheres*2 + noCubes*2];
  vec3 V, L, R;
  //vec3 V2, N2, L2, V3, N3, L3, V4, N4, L4;
  vec3 Ns[1+noSpheres*2 + noCubes*2], Vs[1+noSpheres*2 + noCubes*2], Ls[1+noSpheres*2 + noCubes*2], \
      Rs[1+noSpheres*2 + noCubes*2];
  vec3 intersection1, intersection2, intersection3;
  bool interBool[1+noSpheres*2 + noCubes*2] ;
  vec3 inter[1+noSpheres*2 + noCubes*2] ;

  interBool[0] = intersectFloor(origin, rayDirection, inter[0], dist[0], Vs[0], Ns[0], Ls[0], \
    reflectedColor); 

  interBool[1] = false;

  

  for (int i = 0; i < noSpheres; i++){
    interBool[i+1] = intersectPlane(origin, rayDirection, sphereCenter[i], norPlane, inter[1+i], \
                  dist[1+i], Vs[1+i], Ns[1+i], Ls[1+i]) && \
                  insideSphere(inter[1+i], sphereCenter[i]);
    interBool[1+noSpheres+i] = intersectSphere(origin, rayDirection, sphereCenter[i], inter[1+noSpheres+i], \
                dist[1+noSpheres+i],  Vs[1+noSpheres+i], Ns[1+noSpheres+i], Ls[1+noSpheres+i]) && \
                !frontPlane(inter[1+noSpheres+i], sphereCenter[i], norPlane);
  }

  
  for (int i = 0; i < noCubes; i++){
    interBool[1+noSpheres*2 +i] = intersectPlane(origin, rayDirection, cubeCenter[i], norPlane, \
                  inter[1+noSpheres*2 +i], dist[1+noSpheres*2 +i], Vs[1+noSpheres*2 +i],  \
                  Ns[1+noSpheres*2 +i], Ls[1+noSpheres*2 +i]) && \
                  insideCube(inter[1+noSpheres*2 +i], cubeCenter[i]);

    //interBool[1+noSpheres*2 +i] =false;

    interBool[1+noSpheres*2 + noCubes+i] = intersectCube(origin, rayDirection, i, \
                inter[1+noSpheres*2 + noCubes+i], dist[1+noSpheres*2 + noCubes+i], \
                Vs[1+noSpheres*2 + noCubes+i], Ns[1+noSpheres*2 + noCubes+i], Ls[1+noSpheres*2 + noCubes+i]) 
                && !frontPlane(inter[1+noSpheres*2 + noCubes+i], cubeCenter[i], norPlane);
  }
  
  bool interBoolSol = true;
  for (int i = 0; i < (1+noSpheres*2 + noCubes*2); i++){
    interBoolSol = interBoolSol && !interBool[i];
  }

  if(interBoolSol){
    return false;
  }

  
  float minDist = 1000.0;
  for (int i = 0; i < (1+noSpheres*2 + noCubes*2); i++){
    if(interBool[i] && (minDist > dist[i])){
      intersection = inter[i]+0.0001;
      N = Ns[i];
      if(i==0){
        color = reflectedColor;
        isFloor = true;
      }else{
        isFloor = false;
        if(mode == 0){
          
          float noise = get_nearest_noise(intersection, 0.0, transPerlin, scalePerlin, anglePerlin);
          color = vec3(get_nearest_noise(intersection, -1.0, transPerlin, scalePerlin, anglePerlin), \
                  noise, get_nearest_noise(intersection, 1.0, transPerlin, scalePerlin, anglePerlin));
        }else if(mode == 1){
          float noise = trilinearNoise(intersection, 0.0, transPerlin, scalePerlin, anglePerlin);
          color = vec3(trilinearNoise(intersection, -1.0, transPerlin, scalePerlin, anglePerlin), \
                  noise, trilinearNoise(intersection, 1.0, transPerlin, scalePerlin,anglePerlin));
        }else if(mode == 2){
          //marble
          float noise = trilinearNoise(intersection, 0.0, transPerlin, scalePerlin, anglePerlin);
  
          color = vec3(noise, noise, noise);
        }else if(mode == 3){
          //Grass scale 60, 20, 60
          float noise = trilinearNoise(intersection, 0.0, transPerlin, scalePerlin, anglePerlin);

          color = (vec3(noise*0.2, noise*0.8, noise*0.3)+0.1)*0.6;

          noise = trilinearNoise(intersection, -1.0, transPerlin, scalePerlin, anglePerlin);
          color += (vec3(noise*1.0, noise*0.6, noise*0.2)+0.1)*0.4;
        }else if(mode == 4){
          //Wood scale 4, 80, 4
          float noise = trilinearNoise(intersection, 1.0, transPerlin, scalePerlin, anglePerlin);

          color = (vec3(noise*0.7+0.5, noise*0.4+0.35, noise*0.2+0.2))*0.8;

          noise = trilinearNoise(intersection, -1.0, transPerlin, scalePerlin, anglePerlin);
          color += (vec3(noise*0.2+0.5, noise*0.8+0.3, noise*0.2+0.1))*0.2;
        }else if(mode == 5){
          float noise = trilinearNoise(intersection, 0.0, transPerlin, scalePerlin, anglePerlin);
          color = (vec3(noise)*weightPerlin1 + brightPerlin1);
          
        }else{
          float noise = trilinearNoise(intersection, 1.0, transPerlin, scalePerlin, anglePerlin);
          color = 1.0 - (vec3(noise)*weightPerlin1 + brightPerlin1);

          noise = trilinearNoise(intersection, -1.0, transPerlin1, scalePerlin1, anglePerlin1);
          color += 1.0 - (vec3(noise)*weightPerlin2 + brightPerlin2);

          noise = trilinearNoise(intersection, -1.0, transPerlin2, scalePerlin2, anglePerlin2);
          color += 1.0 - (vec3(noise)*weightPerlin3 + brightPerlin3);


          color = (color/3.0);
        }
      }
      minDist = dist[i];
    }
    
  }
  

  
  if (intersectSomethingGoingToLight(intersection, N)) {
    //reflectedColor = 0.5*color;
    color = 0.5*color;
    shadow = true;
    return true;
  }

  shadow = false;
  
  return true;
}

void main() {
  sphereCenter = vec3[](sphere1) ;
  cubeCenter = vec3[](cube1) ;

  lightPosition = vec3(0.0, 20.0, 0.0);
  vec3 rayDirection = normalize(nearPosition - cameraPosition);


  vec3 intersection1, intersection2;

  cs = 5.0;

  ka = 0.5; 
  kd = 0.4;
  ks = 0.6;
  n = 15.0;

  for(int i=0; i<noCubes; i++){
    A[i] = vec3(cubeCenter[i].x - cs/2.0, cubeCenter[i].y - cs/2.0,cubeCenter[i].z - cs/2.0);
    B[i] = vec3(A[i].x, A[i].y, A[i].z + cs);
    C[i] = vec3(A[i].x + cs, A[i].y, A[i].z + cs);
    D[i] = vec3(A[i].x + cs, A[i].y, A[i].z);
    E[i] = vec3(A[i].x, A[i].y + cs, A[i].z);
    F[i] = vec3(A[i].x, A[i].y + cs, A[i].z + cs);
    G[i] = vec3(A[i].x + cs, A[i].y + cs, A[i].z + cs);
    H[i] = vec3(A[i].x + cs, A[i].y + cs, A[i].z);
  }
 

  Ii = vec3(1.0);
  
  vec3 color, tempColor, colorMax, reflectedColor;
  vec3 N;
  bool shadow = false;
  bool isFloor;

  if (intersectSomething(cameraPosition, rayDirection, intersection1, N, color, reflectedColor,\
     shadow, isFloor)) {
    color += 0.3*tempColor ;
    rayDirection = reflect(rayDirection, N);

    if(reflectingOn){
      isFloor = true;
    }

    if (isFloor && intersectSomething(intersection1, rayDirection, intersection2, N, tempColor, \
        reflectedColor, shadow, isFloor)) {
      color += 0.3*tempColor ;
      rayDirection = reflect(rayDirection, N);

      if(reflectingOn){
        isFloor = true;
      }

      if (isFloor && intersectSomething(intersection2, rayDirection, intersection1, N, tempColor, \
            reflectedColor, shadow, isFloor)) {
        color += 0.3*tempColor ;
      }
    }
    
    fragColor = vec4(color, 1.0); 
  } else {
    fragColor = vec4(0.1, 0.1, 0.1, 1.0);
  }
}

`;
