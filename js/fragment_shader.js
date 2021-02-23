// The Fragment Shader is a program in GLSL. It is responsible
// for the color, basically. (Need to learn more)
//
// This is a template string (notice `)
const fragmentShaderCode = `

precision mediump float;

varying vec3 nearPosition;

uniform vec3 cameraPosition;

uniform vec3 sphereCenter;

uniform vec3 cubeCenter;

uniform vec3 origPlane;

uniform vec3 norPlane;
vec3 A, B, C, D, E, F, G, H;


uniform float floorHeight;
uniform float floorRadius;

float cs;

vec3 lightPosition;
  
vec3 Ii;
float ka, kd, ks;
float n;

bool intersectSphere(vec3 origin, vec3 rayDirection, out vec3 intersection,
                     out float dist,
                     out vec3 V, out vec3 N, out vec3 L) {
  vec3 rayToSphere = sphereCenter - origin;
  float b = dot(rayDirection, -rayToSphere);
  float d = (b * b) - dot(rayToSphere, rayToSphere) + 2.5 * 2.5;

  if (d < 0.0) return false;

  dist = -b - sqrt(d);
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
  dist = (floorHeight - origin.y) / rayDirection.y;
  if (dist < 0.0) return false;

  float x = origin.x + rayDirection.x * dist;
  float z = origin.z + rayDirection.z * dist;
  if (x*x + z*z > floorRadius*floorRadius) return false;

  if (x < 0.0 && z < 0.0) color = vec3(1.0, 1.0, 0.0);
  else if (x < 0.0 && z > 0.0) color = vec3(1.0, 0.0, 1.0);
  else if (x > 0.0 && z < 0.0) color = vec3(0.0, 0.5, 0.5);
  else if (x > 0.0 && z > 0.0) color = vec3(0.5, 0.5, 0.5);

  intersection = vec3(x, floorHeight, z);
  V = -rayDirection;
  N = vec3(0.0, 1.0, 0.0);
  L = normalize(lightPosition - intersection);

  return true;
}

bool intersectPlane(vec3 origin, vec3 rayDirection, out vec3 intersection,
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

bool intersectCube(vec3 origin, vec3 rayDirection, out vec3 intersection,
                   out float dist,
                   out vec3 V, out vec3 N, out vec3 L) {

  if (intersectTriangle(origin, rayDirection, intersection, dist, A, C, B, V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, A, D, C, V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, A, F, E, V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, A, B, F, V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, A, E, H, V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, A, H, D, V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, G, E, F, V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, G, H, E, V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, G, F, B, V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, G, B, C, V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, G, C, D, V, N, L))
    return true;
  else if (intersectTriangle(origin, rayDirection, intersection, dist, G, D, H, V, N, L))
    return true;

  return false;
}

bool insideCube(vec3 origin) {

  vec3 diff = origin - cubeCenter;
  if(abs(diff[0]) > (5.0/2.0) || abs(diff[1]) > (5.0/2.0) || abs(diff[2]) > (5.0/2.0)){
    return false;
  }
  return true;
}

bool insideSphere(vec3 origin) {

  vec3 diff = origin - sphereCenter;
  if(length(diff) > 2.5){
    return false;
  }
  return true;
}
bool frontPlane(vec3 origin) {
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

  bool interCube = intersectCube(origin, rayDirection, intersection, dist, V,N,L) && \
              !frontPlane(intersection);
  if (interCube){
    if (dist < distToLight) return true;
  }
  bool interSphere = intersectSphere(origin, rayDirection, intersection, dist, V,N,L) && \
              !frontPlane(intersection);
  if (interSphere){
    if (dist < distToLight) return true;
  }

  return false;
}

bool intersectSomething(vec3 origin, vec3 rayDirection, out vec3 intersection,
                        out vec3 N, out vec3 color, out vec3 reflectedColor,
                        out bool shadow) {
  float dist[4];
  vec3 V, L, R;
  vec3 V2, N2, L2, V3, N3, L3, V4, N4, L4;
  vec3 Ns[4], Vs[4], Ls[4], Rs[4];
  vec3 intersection1, intersection2, intersection3;
  bool interBool[4] ;
  vec3 inter[4] ;

  interBool[0] = intersectFloor(origin, rayDirection, inter[0], dist[0], Vs[0], Ns[0], Ls[0], reflectedColor); 
  interBool[1] = intersectPlane(origin, rayDirection, inter[1], dist[1], Vs[1], Ns[1], Ls[1]) && \
                (insideSphere(inter[1]) || insideCube(inter[1]));
  interBool[2] = intersectCube(origin, rayDirection, inter[2], dist[2], Vs[2], Ns[2], Ls[2]) && \
              !frontPlane(inter[2]);
  interBool[3] = intersectSphere(origin, rayDirection, inter[3], dist[3], Vs[3], Ns[3], Ls[3]) && \
              !frontPlane(inter[3]);

  if(!interBool[0] && !interBool[1] && !interBool[2] && !interBool[3]){
    return false;
  }

  float minDist = 1000.0;
  for (int i = 0; i < 4; i++)
  {
    if(interBool[i] && (minDist > dist[i])){
      intersection = inter[i];
      N = Ns[i];
      V = Vs[i];
      L = Ls[i];
      if(i==0){
        color = reflectedColor;
      }else{
        color = vec3(sin(intersection[0]), cos(2.0*intersection[1]), sin(3.0*intersection[2]));
        //reflectedColor = vec3(sin(intersection[0]), cos(2.0*intersection[1]), sin(3.0*intersection[2]));
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
  //R = reflect(L, N); 
  //color = colorAt(reflectedColor, V, N, L, R);
  //color = vec3(sin(intersection[0]), cos(2.0*intersection[1]), sin(3.0*intersection[2]));
  return true;
}

void main() {
  lightPosition = vec3(0.0, 20.0, 0.0);
  vec3 rayDirection = normalize(nearPosition - cameraPosition);

  vec3 intersection1, intersection2;

  cs = 5.0;

  ka = 0.5; 
  kd = 0.4;
  ks = 0.6;
  n = 15.0;

  A = vec3(cubeCenter.x - cs/2.0, cubeCenter.y - cs/2.0,cubeCenter.z - cs/2.0);
  B = vec3(A.x, A.y, A.z + cs);
  C = vec3(A.x + cs, A.y, A.z + cs);
  D = vec3(A.x + cs, A.y, A.z);
  E = vec3(A.x, A.y + cs, A.z);
  F = vec3(A.x, A.y + cs, A.z + cs);
  G = vec3(A.x + cs, A.y + cs, A.z + cs);
  H = vec3(A.x + cs, A.y + cs, A.z);

  Ii = vec3(1.0);
  
  vec3 color, tempColor, colorMax, reflectedColor;
  vec3 N;
  bool shadow = false;

  if (intersectSomething(cameraPosition, rayDirection, intersection1, N, color, reflectedColor,\
     shadow)) {
    colorMax = (reflectedColor + vec3(0.7))/1.7;
    
    rayDirection = reflect(rayDirection, N);
    if (!shadow && intersectSomething(intersection1+0.00001, rayDirection, intersection2, N, tempColor, \
        reflectedColor, shadow)) {
      color += 0.3*tempColor ;
      colorMax = (reflectedColor + vec3(0.7))/1.7;
      rayDirection = reflect(rayDirection, N);
      if (!shadow && intersectSomething(intersection2+0.00001, rayDirection, intersection1, N, tempColor, \
            reflectedColor, shadow)) {
        color += 0.3*tempColor ;
      }
    }
    
    gl_FragColor = vec4(color, 1.0); 
  } else {
    gl_FragColor = vec4(0.1, 0.1, 0.1, 1.0);
  }
}

`;
