#version 460 core

in vec4 fragColor;
out vec4 FragColor;

uniform float time;

void main()
{
   FragColor = fragColor * sin(time);
}