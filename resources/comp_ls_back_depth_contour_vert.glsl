#version 110

uniform sampler2D tex0;

void main()
{
  
  gl_TexCoord[0] = gl_MultiTexCoord0;
  gl_Position = ftransform();

}