// The Vertex Shader is a program in OpenGL Shading Language (GLSL)
// It basically is a function that runs on every vertex
//
// This is a template string (notice `)
const vertexShaderCode = `#version 300 es

precision mediump float;

in vec2 vertexPosition;
in vec3 plotPosition;

out vec3 nearPosition;

void main()
{
  gl_Position = vec4(vertexPosition, 1.0, 1.0);
  nearPosition = plotPosition;
}

`;
