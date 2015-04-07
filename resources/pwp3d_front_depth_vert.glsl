#version 110

varying float depth;
varying vec3 N;

void main()
{
    vec4 viewPos = gl_ModelViewMatrix * gl_Vertex;
    N = normalize(gl_NormalMatrix * gl_Normal);
    
    depth = -viewPos.z;
    gl_Position = ftransform();
    
    
}