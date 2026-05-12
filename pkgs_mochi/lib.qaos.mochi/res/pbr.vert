#version 450

layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;
layout(location = 3) in vec2 uv;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec2 fragUV;
layout(location = 2) out vec3 fragPosWorld;
layout(location = 3) out vec3 fragNormalWorld;

layout(push_constant) uniform constant
{
  mat4 model;
} push;

layout(set = 0, binding = 0, row_major) uniform CameraBuffer
{
  mat4 view;
  mat4 proj;
} camera;



void main()
{
  fragColor = color;
  fragUV = uv;

  mat4 trueModel = transpose(push.model);

  vec4 worldPos = trueModel * vec4(pos, 1.0);
  fragPosWorld = worldPos.xyz;

  mat3 normalMatrix = transpose(inverse(mat3(trueModel)));
  fragNormalWorld = normalize(normalMatrix * normal);

  gl_Position = camera.proj * camera.view * worldPos;
}
