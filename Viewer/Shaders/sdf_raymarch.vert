#version 460 core

// sdf_raymarch.vert - full-screen triangle covering the viewport.

layout(location = 0) in vec2 aPos;

out vec2 vUv;

void main() {
    vUv = aPos * 0.5 + 0.5;
    gl_Position = vec4(aPos, 0.0, 1.0);
}
