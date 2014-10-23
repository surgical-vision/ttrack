#version 110

varying float depth;
void main()
{
    vec4 viewPos = gl_ModelViewMatrix * gl_Vertex;
    depth = -viewPos.z;
    gl_Position = ftransform();
}