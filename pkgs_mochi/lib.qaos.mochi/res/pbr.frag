#version 450

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec2 fragUV;
layout(location = 2) in vec3 fragPosWorld;
layout(location = 3) in vec3 fragNormalWorld;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0, row_major) uniform CameraBuffer
{
	mat4 view;
	mat4 proj;
} camera;


struct light_t {
	vec4 pos;
	vec4 color;
};

layout(set = 0, binding = 1) uniform LightBuffer
{
	uvec4 a;
	uvec4 b;
	light_t s[32];
	
} light;



const float METALLIC  = 0.0;
const float ROUGHNESS = 0.9;
const float AMBIENT   = 0.03;

const float PI = 3.14159265359;



float DistributionGGX(vec3 N, vec3 H, float roughness)
{
	float a      = roughness * roughness;
	float a2     = a * a;
	float NdotH  = max(dot(N, H), 0.0);
	float denom  = (NdotH * NdotH * (a2 - 1.0) + 1.0);
	return a2 / (PI * denom * denom);
}

float GeometrySchlickGGX(float NdotX, float roughness)
{
	float r = roughness + 1.0;
	float k = (r * r) / 8.0;
	return NdotX / (NdotX * (1.0 - k) + k);
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
	return GeometrySchlickGGX(max(dot(N, V), 0.0), roughness) * GeometrySchlickGGX(max(dot(N, L), 0.0), roughness);
}

vec3 FresnelSchlick(float cosTheta, vec3 F0)
{
	return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}



void main()
{
	vec3 albedo = fragColor;
	vec3 N      = normalize(fragNormalWorld);
	vec3 camPos = -transpose(mat3(camera.view)) * camera.view[3].xyz;
	vec3 V      = normalize(camPos - fragPosWorld);
	vec3 F0     = mix(vec3(0.04), albedo, METALLIC);

	vec3 Lo = vec3(0.0);

	int activeLightCount = int(light.a.x);
	for(int i = 0; i < activeLightCount; i++) {
		vec3  lightPos       = light.s[i].pos.xyz;
		vec3  lightCol       = light.s[i].color.xyz;
		float lightIntensity = light.s[i].color.w;

		vec3  L           = normalize(lightPos - fragPosWorld);
		vec3  H           = normalize(V + L);
		float dist        = length(lightPos - fragPosWorld);
		vec3  radiance    = lightCol * lightIntensity / (dist * dist);

		float NDF      = DistributionGGX(N, H, ROUGHNESS);
		float G        = GeometrySmith(N, V, L, ROUGHNESS);
		vec3  F        = FresnelSchlick(max(dot(H, V), 0.0), F0);

		vec3  specular = (NDF * G * F) / (4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.0001);
		vec3 kD = (1.0 - F) * (1.0 - METALLIC);

		Lo += (kD * albedo / PI + specular) * radiance * max(dot(N, L), 0.0);
	}

	vec3 color = AMBIENT * albedo + Lo;
	color = color / (color + vec3(1.0));
	color = pow(color, vec3(1.0 / 2.2));

	outColor = vec4(color, 1.0);
}
