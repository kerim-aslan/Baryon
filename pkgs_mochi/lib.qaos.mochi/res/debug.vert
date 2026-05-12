#version 450

layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;
layout(location = 3) in vec2 uv;

layout(push_constant) uniform constant
{
  mat4 model;
} push;

layout(set = 0, binding = 0) uniform CameraBuffer
{
  mat4 view;
  mat4 proj;
} camera;

void main()
{
  mat4 trueModel = transpose(push.model);
  mat4 trueView = transpose(camera.view);
  mat4 trueProj = transpose(camera.proj);

  vec4 worldPos = trueModel * vec4(pos, 1.0);
  gl_Position = trueProj * trueView * worldPos;
}
